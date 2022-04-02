# Mediator/Observer parser processing

Někdy potřebujeme vytvořit implementaci  

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