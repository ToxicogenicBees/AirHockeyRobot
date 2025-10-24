#include "main.h"

// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);

int main() {
    // Initialize the controller
    HAL_Init();
    // SystemClock_Config();
    // MX_GPIO_Init();

    // Initialize sensors
    x_dist.init();
    y_dist.init();
    x_limit.init();
    y_limit.init();

    // Main loop
    while(true) {

    }
}