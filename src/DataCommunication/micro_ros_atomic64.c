#include <stdint.h>

#include <zephyr/kernel.h>

static struct k_spinlock atomic64_lock;

uint64_t __atomic_load_8(uint64_t *address, int memory_order)
{
    ARG_UNUSED(memory_order);
    k_spinlock_key_t key = k_spin_lock(&atomic64_lock);
    uint64_t value = *address;
    k_spin_unlock(&atomic64_lock, key);
    return value;
}

void __atomic_store_8(uint64_t *address, uint64_t value, int memory_order)
{
    ARG_UNUSED(memory_order);
    k_spinlock_key_t key = k_spin_lock(&atomic64_lock);
    *address = value;
    k_spin_unlock(&atomic64_lock, key);
}

uint64_t __atomic_exchange_8(uint64_t *address, uint64_t value, int memory_order)
{
    ARG_UNUSED(memory_order);
    k_spinlock_key_t key = k_spin_lock(&atomic64_lock);
    uint64_t previous = *address;
    *address = value;
    k_spin_unlock(&atomic64_lock, key);
    return previous;
}

uint64_t __atomic_fetch_add_8(uint64_t *address, uint64_t value, int memory_order)
{
    ARG_UNUSED(memory_order);
    k_spinlock_key_t key = k_spin_lock(&atomic64_lock);
    uint64_t previous = *address;
    *address += value;
    k_spin_unlock(&atomic64_lock, key);
    return previous;
}
