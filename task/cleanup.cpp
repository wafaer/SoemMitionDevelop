//
// Created by wg on 4/29/26.
//
#include <cstdlib>

#include "motion/motion.h"
#include "hal/hpsocket.h"
#include "hal/ServoEnable.h"

static void cleanup_all(void)
{
    ecat_thread_exit();
    socket_thread_exit();
    motion_thread_exit();
}

// Constructor runs before main(); registers cleanup_all with atexit so it
// fires on every exit() call (normal, signal-driven, or error path).
struct CleanupRegistrar {
    CleanupRegistrar() { std::atexit(cleanup_all); }
};

static CleanupRegistrar g_registrar;