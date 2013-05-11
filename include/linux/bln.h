/* include/linux/bln.h */

#ifndef _LINUX_BLN_H
#define _LINUX_BLN_H

struct bln_implementation {
    void (*enable)(void);
    void (*disable)(void);
};

void register_bln_implementation(struct bln_implementation *imp);
bool bln_is_ongoing(void);
void bln_wakelock_destroy(void);
void bln_wakelock_acquire(void);
void bln_wakelock_release(void);
#endif
