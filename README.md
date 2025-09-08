# Button Component for ESP-IDF
FreeRTOS-based button driver for ESP32, designed to handle GPIO button presses with:
- **Debounce filtering**
- **ISR support**
- **Callback mechanisms**

## Features
- **Debounce Filtering**: Configurable debounce time to prevent false triggers.
- **Interrupt Handling**: Efficient ISR-based button event detectio
- **Callbacks**: User-defined callbacks for button press/release events.
- **Flexible GPIO Configuration**: Supports pull-up, pull-down, or floating pins.
- **FreeRTOS Integration**: Handles button events via a dedicated FreeRTOS task and queue.

## How to Use
### Hardware
- **ESP32 Board**: Any ESP32-based development board.
- **Button **: Any button.

### Connection Diagram

#### Basic Button Connection (Most Common)
```
ESP32 GPIO ── Button ── GND
     │
  Pull-up (internal)
```

#### Using BOOT Button
```
Just press the BOOT button on your ESP32 board!
No additional wiring required.
```

### Getting Started
1. Install ESP-IDF<br>
 Follow the ESP-IDF installation guide for your operating system.  
2. Clone the Repository
   ```sh
    git clone https://github.com/felipegtralli/esp-button.git
    ```
3. Add esp-button as a Component<br>
 Include the esp-button library in your ESP-IDF project by placing it in the components directory or by linking it via an idf_component.yml.
4. Reconfigure
   ```sh
   idf.py reconfigure
   ```
5. Build and Flash
    ```sh
    idf.py build flash
    ```
    
## Testing Component
1. Run Example Code<br>
 Flash and monitor any example provided in the examples folder.
    ```sh
    idf.py build flash monitor
    ```
