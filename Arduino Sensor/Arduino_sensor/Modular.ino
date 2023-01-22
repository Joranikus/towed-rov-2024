// expansion ports
void send_sensor() {
  if (i2c_variable != "") {
      Serial.println("<" + i2c_variable + ":" + "trybe" + ">");
  }
  if (software_uart != "") {
    if (software_serial.available()) {
      Serial.println("<" + software_uart + ":" + software_serial.read() + ">");
    }
  }
  if (a0 != "") {
    Serial.println("<" + a0 + ":" + analogRead(A0) + ">");
  }
  if (a1 != "") {
    Serial.println("<" + a1 + ":" + analogRead(A1) + ">");
  }
  if (a2 != "") {
    Serial.println("<" + a2 + ":" + analogRead(A2) + ">");
  }
  if (a3 != "") {
    Serial.println("<" + a3 + ":" + analogRead(A3) + ">");
  }
  if (d2 != "") {
    Serial.println("<" + d2 + ":" + digitalRead(D2) + ">");
  }
  if (d3 != "") {
    Serial.println("<" + d3 + ":" + digitalRead(D3) + ">");
  }
  if (d4 != "") {
    Serial.println("<" + d4 + ":" + digitalRead(D4) + ">");
  }
}
