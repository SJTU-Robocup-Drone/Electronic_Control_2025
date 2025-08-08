const int magnetPins[] = {1, 2, 14}; //电磁铁引脚，后续可根据需要更改

void setup()
{
  Serial.begin(115200);
  pinMode(magnetPins[0],OUTPUT);
  pinMode(magnetPins[1],OUTPUT);
  pinMode(magnetPins[2],OUTPUT);
//将三个电磁铁引脚设置为输出模式
  digitalWrite(magnetPins[0],HIGH);
  digitalWrite(magnetPins[1],HIGH);
  digitalWrite(magnetPins[2],HIGH);
//初始状态设置为HIGH
  Serial.println("Magnet control ready. Send 1/0 command via Serial.");
}

void loop() {
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n'); // 读取字符串命令，假设命令开头是"1"或"2"或"3"，中间是1或0（表示高低电平），结尾是'\n'
    int value = cmd.toInt();
    switch(value){
      case 10:
      digitalWrite(magnetPins[0],LOW);
      break;
      case 11:
      digitalWrite(magnetPins[0],HIGH);
      break;
      case 20:
      digitalWrite(magnetPins[1],LOW);
      break;
      case 21:
      digitalWrite(magnetPins[1],HIGH);
      break;
      case 30:
      digitalWrite(magnetPins[2],LOW);
      break;
      case 31:
      digitalWrite(magnetPins[2],HIGH);
      default:
      Serial.println("Invalid input\n");
    }
  }
}