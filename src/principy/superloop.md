# Supersmyčka a regulace v ní

```c++
const int T = 30;
long long ms = millis() + T;
while(rum) {
    comm.send(sensor.BuildReads());
    comm.send(drive.BuildReads());
    
    while (ms < millis())
      comm.loop();
      
    ms = millis() + T;
    
    sensor.ProcessLineSensors();
    drive.ComputeOdometry();
    
    controller.Control();                                                       
    
    drive.ComputeMotorRamps();        
    comm.send(sensor.BuildWrites());
    comm.send(drive.BuildWrites());
  }
```