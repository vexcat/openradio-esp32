# OpenRadio (ESP32)

This is the OpenRadio repeater for the ESP32. It passes OpenRadio packets from the V5 brain to any device connected to the same network as the ESP32. It is currently being refactored as the previous networking code was pretty buggy and used two threads per connection, so the code might not work out-of-the-box.

For more details on OpenRadio, see https://ungato.tk/radio.