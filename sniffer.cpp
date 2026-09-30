// OLED I2C sniffer for the Daisy Pod.
//
// Passively listens to an I2C bus (e.g. a synth's SSD1306 OLED) and streams
// every transaction to the computer over the Pod's micro-USB port (USB CDC).
// The pins are plain inputs with no pull-ups; the Pod never drives the bus.
//
// Wiring (tap into the target's OLED lines):
//   Pod Seed D11 (pin 12) <- SCL
//   Pod Seed D12 (pin 13) <- SDA
//   Pod GND               <- GND   (required)
//
// LEDs: LED1 green = bus activity, LED2 red = bus data may have been lost
//       recently (event ring overflow, or an interrupt serviced too late).
#include "daisy_pod.h"
#include "usbd_def.h"

using namespace daisy;

extern "C" USBD_HandleTypeDef hUsbDeviceFS; // defined in libDaisy hid/usb.cpp

DaisyPod hw;

// D11 = PB8 (SCL), D12 = PB9 (SDA). Both land on EXTI9_5.
constexpr uint32_t SCL_PIN = GPIO_PIN_8;
constexpr uint32_t SDA_PIN = GPIO_PIN_9;

// ---------------------------------------------------------------------------
// Bus events: written by the EXTI ISR, drained by the main loop.
// ---------------------------------------------------------------------------
constexpr uint16_t EV_BYTE  = 0x0800; // low 8 bits = byte value
constexpr uint16_t EV_NACK  = 0x0100; // set with EV_BYTE when the 9th bit was high
constexpr uint16_t EV_START = 0x1000;
constexpr uint16_t EV_STOP  = 0x2000;

constexpr uint32_t RING_SIZE = 16384; // power of two
static volatile uint16_t ring[RING_SIZE];
static volatile uint32_t ring_head = 0; // ISR writes
static volatile uint32_t ring_tail = 0; // main loop reads

// ISR decoder state
static bool     in_txn = false;
static uint32_t nbits  = 0;
static uint32_t shift  = 0;

// Counters (reported to the PC)
static volatile uint32_t stat_ring_overflow = 0;
static volatile uint32_t stat_late_irq      = 0;
static uint32_t          stat_txns          = 0;
static uint32_t          stat_bytes         = 0;
static uint32_t          stat_usb_drops     = 0;

static inline void Push(uint16_t ev)
{
    const uint32_t h = ring_head;
    if(h - ring_tail >= RING_SIZE)
    {
        stat_ring_overflow = stat_ring_overflow + 1;
        return;
    }
    ring[h & (RING_SIZE - 1)] = ev;
    ring_head                 = h + 1;
}

extern "C" void EXTI9_5_IRQHandler(void)
{
    // Sample the pins first: this is the timing-critical part.
    const uint32_t idr = GPIOB->IDR;
    const uint32_t pr  = __HAL_GPIO_EXTI_GET_IT(SCL_PIN | SDA_PIN);
    __HAL_GPIO_EXTI_CLEAR_IT(pr);

    const bool scl = idr & SCL_PIN;
    const bool sda = idr & SDA_PIN;

    // Both edges pending means we were serviced too late to know their order.
    if(pr == (SCL_PIN | SDA_PIN))
        stat_late_irq = stat_late_irq + 1;

    if(pr & SCL_PIN) // SCL rising edge: SDA holds a valid bit
    {
        if(in_txn)
        {
            if(nbits < 8)
            {
                shift = (shift << 1) | (sda ? 1u : 0u);
                nbits++;
            }
            else // 9th clock is the ACK bit
            {
                Push(EV_BYTE | (shift & 0xFF) | (sda ? EV_NACK : 0));
                nbits = 0;
                shift = 0;
            }
        }
    }

    // SDA changing while SCL is high = START (falling) or STOP (rising).
    if((pr & SDA_PIN) && scl)
    {
        Push(sda ? EV_STOP : EV_START);
        in_txn = !sda;
        nbits  = 0;
        shift  = 0;
    }
}

static void InitBusTap()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gi = {};
    gi.Pull             = GPIO_NOPULL;
    gi.Speed            = GPIO_SPEED_FREQ_LOW;

    gi.Pin  = SCL_PIN;
    gi.Mode = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(GPIOB, &gi);

    gi.Pin  = SDA_PIN;
    gi.Mode = GPIO_MODE_IT_RISING_FALLING;
    HAL_GPIO_Init(GPIOB, &gi);

    __HAL_GPIO_EXTI_CLEAR_IT(SCL_PIN | SDA_PIN);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

// ---------------------------------------------------------------------------
// USB output. Packet format (little endian):
//   0xA5 0x5A  type  len_lo len_hi  payload[len]  checksum
//   checksum = 8-bit sum of type, len bytes and payload.
//
//   type 0x01 (I2C chunk): payload = flags, bytes...
//     flags bit0 FIRST: chunk starts a transaction (bytes[0] = address byte)
//     flags bit1 LAST:  transaction ended (STOP or repeated START)
//     flags bit2 NACK:  a byte in this chunk was not acknowledged
//   type 0x02 (stats): payload = u32 txns, bytes, ring_overflow, usb_drops,
//                                late_irq
// ---------------------------------------------------------------------------
constexpr uint8_t PKT_I2C   = 0x01;
constexpr uint8_t PKT_STATS = 0x02;

constexpr uint8_t FLAG_FIRST = 0x01;
constexpr uint8_t FLAG_LAST  = 0x02;
constexpr uint8_t FLAG_NACK  = 0x04;

constexpr size_t TX_BUF_SIZE = 4096;
static uint8_t   tx_buf[2][TX_BUF_SIZE];
static size_t    tx_len    = 0;
static int       tx_active = 0; // buffer being filled; the other may be in flight

static void FlushUsb()
{
    // CDC_Transmit_FS dereferences the class data without a NULL check, and
    // it only exists once a host has configured the device.
    if(tx_len == 0 || hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED
       || hUsbDeviceFS.pClassData == nullptr)
        return;
    if(hw.seed.usb_handle.TransmitInternal(tx_buf[tx_active], tx_len)
       == UsbHandle::Result::OK)
    {
        tx_active ^= 1;
        tx_len = 0;
    }
}

static bool SendPacket(uint8_t type, const uint8_t* payload, size_t len)
{
    const size_t total = len + 6;
    if(tx_len + total > TX_BUF_SIZE)
    {
        FlushUsb();
        if(tx_len + total > TX_BUF_SIZE)
        {
            stat_usb_drops++;
            return false;
        }
    }
    uint8_t* p   = &tx_buf[tx_active][tx_len];
    uint8_t  sum = type + uint8_t(len) + uint8_t(len >> 8);
    *p++         = 0xA5;
    *p++         = 0x5A;
    *p++         = type;
    *p++         = uint8_t(len);
    *p++         = uint8_t(len >> 8);
    for(size_t i = 0; i < len; i++)
    {
        *p++ = payload[i];
        sum += payload[i];
    }
    *p++ = sum;
    tx_len += total;
    return true;
}

// ---------------------------------------------------------------------------
// Transaction assembly
// ---------------------------------------------------------------------------
constexpr size_t CHUNK_MAX = 1024;
static uint8_t   chunk[CHUNK_MAX + 1]; // [0] = flags
static size_t    chunk_len  = 0;       // data bytes in chunk
static bool      chunk_open = false;   // inside a transaction
static bool      activity   = false;

static void EmitChunk(bool last)
{
    if(!chunk_open)
        return;
    if(last)
        chunk[0] |= FLAG_LAST;
    SendPacket(PKT_I2C, chunk, chunk_len + 1);
    chunk[0]  = 0;
    chunk_len = 0;
    if(last)
    {
        chunk_open = false;
        stat_txns++;
    }
}

static void DrainEvents()
{
    while(ring_tail != ring_head)
    {
        const uint16_t ev = ring[ring_tail & (RING_SIZE - 1)];
        ring_tail         = ring_tail + 1;

        if(ev & EV_START)
        {
            EmitChunk(true); // repeated START ends the previous transaction
            chunk_open = true;
            chunk[0]   = FLAG_FIRST;
            chunk_len  = 0;
        }
        else if(ev & EV_STOP)
        {
            EmitChunk(true);
        }
        else if((ev & EV_BYTE) && chunk_open)
        {
            chunk[1 + chunk_len++] = uint8_t(ev);
            if(ev & EV_NACK)
                chunk[0] |= FLAG_NACK;
            stat_bytes++;
            activity = true;
            if(chunk_len == CHUNK_MAX)
                EmitChunk(false);
        }
    }
}

static void SendStats()
{
    uint32_t v[5] = {stat_txns,
                     stat_bytes,
                     stat_ring_overflow,
                     stat_usb_drops,
                     stat_late_irq};
    uint8_t  payload[sizeof(v)];
    for(size_t i = 0; i < 5; i++)
        for(size_t b = 0; b < 4; b++)
            payload[i * 4 + b] = uint8_t(v[i] >> (8 * b));
    SendPacket(PKT_STATS, payload, sizeof(payload));
}

static void LowerUsbIrqPriority()
{
    // libDaisy registers every IRQ at priority 0. The bus tap must be able to
    // preempt USB, otherwise a USB interrupt can hide SCL edges from us.
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 2, 0);
    HAL_NVIC_SetPriority(OTG_FS_EP1_OUT_IRQn, 2, 0);
    HAL_NVIC_SetPriority(OTG_FS_EP1_IN_IRQn, 2, 0);
}

int main(void)
{
    hw.Init();
    hw.seed.usb_handle.Init(UsbHandle::FS_INTERNAL);
    LowerUsbIrqPriority();
    InitBusTap();

    uint32_t last_stats = System::GetNow();
    uint32_t last_led   = last_stats;
    uint32_t act_until  = 0;
    uint32_t err_until  = 0;
    uint32_t last_errs  = 0;

    while(1)
    {
        DrainEvents();
        FlushUsb();

        const uint32_t now = System::GetNow();
        if(now - last_stats >= 500)
        {
            last_stats = now;
            SendStats();
        }

        if(now - last_led >= 10)
        {
            last_led = now;
            if(activity)
            {
                activity  = false;
                act_until = now + 50;
            }
            // USB drops are left out: they also happen whenever no viewer is
            // connected, and the viewer displays them.
            const uint32_t errs = stat_ring_overflow + stat_late_irq;
            if(errs != last_errs)
            {
                last_errs = errs;
                err_until = now + 500;
            }
            hw.led1.Set(0.f, int32_t(act_until - now) > 0 ? 1.f : 0.f, 0.f);
            hw.led2.Set(int32_t(err_until - now) > 0 ? 1.f : 0.f, 0.f, 0.f);
            hw.UpdateLeds();
        }
    }
}
