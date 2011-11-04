/*
 * include/linux/input/sec-input-bridge.h
 *
 * Copyright 2011 Samsung electronics.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __SEC_INPUT_BRIDGE_H__
#define __SEC_INPUT_BRIDGE_H__

enum mkey_check_option {
    MKEY_CHECK_AUTO,
    MKEY_CHECK_AWAYS
};

struct sec_input_bridge_mkey {
    unsigned int type;
    unsigned int code;
    enum mkey_check_option option;
};

struct sec_input_bridge_platform_data {
    const struct sec_input_bridge_mkey *mkey_map;
    unsigned int num_mkey;
};

#endif  //__SEC_INPUT_BRIDGE_H__
