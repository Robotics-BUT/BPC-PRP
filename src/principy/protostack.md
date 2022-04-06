# Protokolový stack

```c++
void loop() 
{
  auto [sender, received] = udp.receive();                                      // udp -> buffer
  if (sender.empty())
    return;
  auto rpacket = from_buffer(received);                                         // buffer -> transport
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
   auto transmit = to_buffer(tpacket);                                          // transport -> buffer
   udp.send(sender, transmit);                                                  // buffer -> udp
}
```