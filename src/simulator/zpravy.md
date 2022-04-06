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

```c++
std::vector<Msg> ProcessIncoming(const Msg &msg) {
  auto res = map.find(msg[0]);
  if (res == map.end())
    return {}
    
  return (res->second)(msg);
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