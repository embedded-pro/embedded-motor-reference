# Performance Optimization Guide for Embedded Systems

This guide documents best practices for writing high-performance code targeting ARM Cortex-M microcontrollers, with specific focus on real-time motor control applications.

## Table of Contents

1. [Compiler Optimization Fundamentals](#compiler-optimization-fundamentals)
2. [Writing Optimization-Friendly Code](#writing-optimization-friendly-code)
3. [Debug Mode Performance](#debug-mode-performance)
4. [Analyzing Generated Code](#analyzing-generated-code)
5. [Measuring Performance](#measuring-performance)
6. [Common Pitfalls](#common-pitfalls)

---

## Compiler Optimization Fundamentals

### Optimization Levels

| Flag  | Description                 | Use Case                          |
|-------|-----------------------------|-----------------------------------|
| `-O0` | No optimization             | Default debug, full debuggability |
| `-Og` | Debug-friendly optimization | **Recommended for debug builds**  |
| `-O1` | Basic optimization          | Faster compile, moderate speed    |
| `-O2` | Standard optimization       | Good balance of speed/size        |
| `-O3` | Aggressive optimization     | Maximum speed, may increase size  |
| `-Os` | Size optimization           | Flash-constrained systems         |

### Critical Flags for Embedded

```cmake
# Recommended flags for ARM Cortex-M4F
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
    -mcpu=cortex-m4 \
    -mfpu=fpv4-sp-d16 \
    -mfloat-abi=hard \
    -mthumb \
    -ffunction-sections \
    -fdata-sections")

# Linker flags to remove unused code
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")
```

### Fast-Math Considerations

```cpp
// Enables aggressive floating-point optimizations
// WARNING: May change numerical behavior slightly
#pragma GCC optimize("fast-math")
```

Effects of `-ffast-math`:
- Assumes no NaN or Infinity
- Allows reordering of operations
- Enables FMA (Fused Multiply-Add) instructions
- May break IEEE 754 compliance

**Use only when**: You control all inputs and don't need strict IEEE behavior.

---

## Writing Optimization-Friendly Code

### 1. Avoid Virtual Functions in Hot Paths

**Bad** (virtual dispatch overhead):
```cpp
class ITrigonometry {
public:
    virtual float Sine(float angle) const = 0;
};

// In hot path:
float sin_val = trig->Sine(angle);  // vtable lookup + indirect call
```

**Good** (static dispatch):
```cpp
struct FastTrigonometry {
    static inline float Sine(float angle) noexcept {
        // Direct call, can be inlined
        return LookupTable[index];
    }
};

// In hot path:
float sin_val = FastTrigonometry::Sine(angle);  // Inlined
```

### 2. Avoid `std::optional` in Performance-Critical Code

**Bad** (generates has_value() checks):
```cpp
std::optional<float> setPoint;

void Process(float input) {
    if (!setPoint.has_value())  // Extra branch + memory access
        return;
    // ...
}
```

**Good** (simple flag):
```cpp
float setPointValue = 0.0f;
bool hasSetPoint = false;

void Process(float input) {
    if (!hasSetPoint) [[unlikely]]
        return;
    // ...
}
```

### 3. Use `constexpr` and `inline` Aggressively

```cpp
// Computed at compile time
inline constexpr std::array<float, 512> sineLUT = []() {
    std::array<float, 512> table{};
    for (size_t i = 0; i < 512; ++i)
        table[i] = std::sin(2.0f * M_PI * i / 512.0f);
    return table;
}();
```

### 4. Use Compiler Attributes

```cpp
// Force inlining even without optimization
#define ALWAYS_INLINE __attribute__((always_inline)) inline

// Mark hot functions for better code placement
#define HOT_FUNCTION __attribute__((hot))

// Combined macro for critical functions
#define OPTIMIZE_FOR_SPEED \
    __attribute__((always_inline, hot, optimize("-O3"), optimize("-ffast-math"))) inline
```

### 5. Prefer Fixed-Size Types

```cpp
// Good: Explicit sizes, portable
uint32_t counter;
int16_t current_mA;
float voltage_V;

// Avoid: Implementation-defined sizes
int counter;
short current;
```

### 6. Minimize Stack Usage

```cpp
// Bad: Large stack allocation
void Calculate() {
    float buffer[1024];  // 4KB on stack!
    // ...
}

// Good: Static or class member
class Calculator {
    static float buffer[1024];  // In .bss section
    // ...
};
```

### 7. Use FMA When Possible

The compiler generates FMA (Fused Multiply-Add) instructions with `-ffast-math`:

```cpp
// This pattern:
result = a * b + c;

// Becomes single instruction:
// vfma.f32 s0, s1, s2  (1 cycle instead of 2)
```

---

## Debug Mode Performance

### Problem

By default, Debug builds (`-O0`) disable all optimizations, making code 3-10x slower than Release. This is problematic for:
- Real-time control loops (FOC, PID)
- Interrupt service routines
- Communication protocols with timing requirements

### Solution 1: Use `-Og` for Debug Builds

```cmake
# In CMakeLists.txt
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g" CACHE STRING "Debug flags" FORCE)
```

`-Og` provides:
- Basic inlining
- Dead code elimination
- Register allocation
- Still debuggable (variable inspection works)

### Solution 2: Per-File Optimization Pragmas

```cpp
// At the top of performance-critical .cpp files
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

// Rest of implementation...
```

### Solution 3: Per-Function Attributes

```cpp
__attribute__((optimize("-O3")))
void CriticalFunction() {
    // This function is always optimized
}
```

**Note**: Function-level attributes don't propagate to callees. Use file-level pragmas for better results.

---

## Analyzing Generated Code

### Disassembly with objdump

```bash
# Basic disassembly
arm-none-eabi-objdump -d firmware.elf > disassembly.txt

# With C++ demangling
arm-none-eabi-objdump -d -C firmware.elf > disassembly.txt

# Specific function (grep pattern)
arm-none-eabi-objdump -d -C firmware.elf | grep -A 100 "FunctionName"

# From static library
arm-none-eabi-objdump -d -C libfoo.a | grep -A 50 "ClassName::Method"

# Include source interleaved (requires -g)
arm-none-eabi-objdump -d -S -C firmware.elf > disassembly_with_source.txt
```

### Size Analysis

```bash
# Section sizes
arm-none-eabi-size firmware.elf

# Detailed symbol sizes (sorted by size)
arm-none-eabi-nm --size-sort -C firmware.elf

# Top 20 largest symbols
arm-none-eabi-nm --size-sort -C firmware.elf | tail -20
```

### Reading Assembly Output

Key ARM Cortex-M4F instructions to look for:

| Instruction | Meaning                 | Cycles |
|-------------|-------------------------|--------|
| `vfma.f32`  | Fused multiply-add      | 1      |
| `vmul.f32`  | Multiply                | 1      |
| `vadd.f32`  | Add                     | 1      |
| `vdiv.f32`  | Division                | 14     |
| `vsqrt.f32` | Square root             | 14     |
| `blx r3`    | Indirect call (virtual) | 3+     |
| `bl <addr>` | Direct call             | 1+N    |
| `push/pop`  | Stack operations        | 1-2    |

### Signs of Poor Optimization

```asm
; Bad: Excessive stack operations
push    {r4, r5, r6, r7, r8, r9, r10, r11, lr}
sub     sp, #104        ; Large stack frame

; Bad: Virtual dispatch
ldr     r3, [r0, #0]    ; Load vtable pointer
ldr     r3, [r3, #4]    ; Load function pointer
blx     r3              ; Indirect call

; Bad: Repeated memory loads
ldr     r3, [r7, #4]    ; Same address loaded
; ... some code ...
ldr     r3, [r7, #4]    ; Again!
```

### Signs of Good Optimization

```asm
; Good: Minimal stack usage
push    {r4, r5, lr}
sub     sp, #16

; Good: FMA instructions
vfma.f32  s0, s1, s2

; Good: Conditional execution (no branches)
vcmpe.f32 s0, s1
it        gt
vmovgt.f32 s0, s1

; Good: Loop unrolling
vldr    s0, [r0, #0]
vldr    s1, [r0, #4]
vldr    s2, [r0, #8]
vldr    s3, [r0, #12]
```

---

## Measuring Performance

### Cycle Counter (DWT)

```cpp
// Enable cycle counter (do once at startup)
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CYCCNT = 0;
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

// Measure cycles
uint32_t start = DWT->CYCCNT;
CriticalFunction();
uint32_t cycles = DWT->CYCCNT - start;
```

### GPIO Toggle Method

```cpp
// Simple but requires oscilloscope
GPIO_SetPin(DEBUG_PIN);
CriticalFunction();
GPIO_ClearPin(DEBUG_PIN);
// Measure pulse width on scope
```

### Timer-Based Measurement

```cpp
// Using SysTick or hardware timer
uint32_t start = SysTick->VAL;
CriticalFunction();
uint32_t elapsed = start - SysTick->VAL;  // SysTick counts down
```

### Typical Cycle Budgets (120 MHz Cortex-M4)

| Control Loop Rate | Available Cycles |
|-------------------|------------------|
| 10 kHz            | 12,000 cycles    |
| 20 kHz            | 6,000 cycles     |
| 40 kHz            | 3,000 cycles     |
| 100 kHz           | 1,200 cycles     |

**FOC typical requirements**: 200-400 cycles (optimized), 800-1500 cycles (unoptimized)

---

## Common Pitfalls

### 1. Heap Allocation

```cpp
// NEVER in embedded hot paths
auto ptr = std::make_unique<Data>();  // malloc!
std::vector<float> buffer;            // malloc!
std::string message;                  // malloc!
```

### 2. Exception Handling Overhead

```cpp
// Compile with: -fno-exceptions -fno-rtti
// Avoid try/catch in embedded code
```

### 3. printf/iostream in ISR

```cpp
// NEVER in interrupt handlers
void ISR_Handler() {
    printf("Debug: %f\n", value);  // ~10,000+ cycles!
}
```

### 4. Floating-Point in Integer-Only Code

```cpp
// Bad: Promotes to float
int result = value * 1.5;

// Good: Integer-only
int result = value * 3 / 2;
```

### 5. Unaligned Access

```cpp
// Potential unaligned access (may cause fault or slow access)
struct __attribute__((packed)) BadStruct {
    uint8_t a;
    uint32_t b;  // Unaligned!
};

// Good: Natural alignment
struct GoodStruct {
    uint32_t b;
    uint8_t a;
    uint8_t padding[3];
};
```

---

## Quick Reference Card

### GCC Optimization Pragmas
```cpp
#pragma GCC optimize("O3")           // Maximum speed
#pragma GCC optimize("Os")           // Minimum size  
#pragma GCC optimize("fast-math")    // Aggressive FP
#pragma GCC push_options             // Save current options
#pragma GCC pop_options              // Restore options
```

### Function Attributes
```cpp
__attribute__((always_inline))       // Force inline
__attribute__((noinline))            // Prevent inline
__attribute__((hot))                 // Optimize for speed
__attribute__((cold))                // Optimize for size
__attribute__((pure))                // No side effects
__attribute__((const))               // Pure + no memory reads
__attribute__((flatten))             // Inline all callees
```

### Branch Hints
```cpp
if (condition) [[likely]] { }        // C++20
if (condition) [[unlikely]] { }      // C++20
if (__builtin_expect(condition, 1))  // GCC
```

### Useful objdump Commands
```bash
# Full disassembly with source
arm-none-eabi-objdump -d -S -C file.elf

# Just .text section
arm-none-eabi-objdump -d -j .text file.elf

# Show relocations
arm-none-eabi-objdump -d -r file.o

# Section headers
arm-none-eabi-objdump -h file.elf
```

---

## References

- [ARM Cortex-M4 Technical Reference Manual](https://developer.arm.com/documentation/100166/latest/)
- [GCC Optimization Options](https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html)
- [ARM Compiler armasm User Guide](https://developer.arm.com/documentation/dui0473/latest)
- [Embedded Artistry - Embedded C++ Guidelines](https://embeddedartistry.com/)

---
