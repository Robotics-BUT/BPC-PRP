# Aplikační protokol

Obsluha protokolu

```c++
void loop() 
{
  auto [sender, received] = udp.receive();                                      // udp -> mezivrstva
  if (sender.empty())
    return;
  auto rpacket = from_buffer(received);                                         // mezivrstva -> transport
  std::vector<Msg> replies;    
  for (auto &request : ParseAll(rpacket)) {                                     // transport -> aplikace
    replies.append(ProcessIncoming(request));                                   // obsluha aplikacniho protokolu
  }
  if (!replies.empty()) {
    auto tpacket = BuildAll(replies);                                           // aplikace -> transport
    auto transmit = to_buffer(tpacket);                                         // transport -> mezivrstva
    udp.send(sender, transmit);                                                 // mezivrstva -> udp 
  }
}

void send(std::vector<Msg> msgs) 
{
   auto tpacket = BuildAll(msgs);                                               // aplikace -> transport
   auto transmit = to_buffer(tpacket);                                          // transport -> mezivrstva
   udp.send(sender, transmit);                                                  // mezivrstva -> udp
}
```
Principy aneb hloupeko kopni a chytreho nakopni

Deferred processing

```c++
std::vector<Msg> ProcessIncoming(const Msg &msg) {
  auto res = map.find(msg[0]);
  if (res == map.end())
    return {}
    
  return (res->second)(msg);
}
```

Data pro popis stavu robotu

```c++
std::vector<Msg> MyRobotData::ODOReceived(const Msg &msg)
{
  if (msg.size() != 3)
    return {};
    
  OdoL = std::stod(msg[1]);
  OdoR = std::stod(msg[2]);  
  return {};
}

std::vector<Msg> MyRobotData::BuildReads() 
{
  return {
    {"ODO"},
    {"SENSOR","1"},
    {"SENSOR","2"},
    };
}

std::vector<Msg> MyRobotData::BuildWrites() 
{
  return {
    {"SPEED", std::to_string(LeftSpeed), std::string(RightSpeed)},
    {"LED"},
    };
}
```

Controller pro stavove ovladani robotu

```c++
class Controller {
  using State = void(Controller::*)();
public:
  State state{&Controller::DoSearchLine}
}
  

void Controller::DoSearchLine() 
{
  if (!robot.LineMissing)
    state = &Controller::DoFollowLine;
}  

void Controller::DoFollowLine()
{
    robot.SenseLine();
    robot.RegulateLine();
    if (robot.LineMissing)
      state = &Controller::DoTryMissingLine;
}

void Controller::DoTryMissingLine()
{
    robot.SenseLine();
    if (!robot.LineMissing)
       state = &Controller::DoFollowLine;
}
```

Supersmycka

```c++
  controller.state = &Controller::DoSearchLine

  const int T = 20;
  long long ms = millis + T;
  while(rum) {
    send(data.Reads());
    
    while (ms < millis())
      loop();
    ms = millis() + T;
    
    (controller->*state)();
    
    robot.ComputeMotorRamps();    
    
    send(data.Writes());
  }

```