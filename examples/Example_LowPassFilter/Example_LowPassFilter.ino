const float alpha = 0.5;
double data_filtered[] = {0, 0};
const int n = 1;
const int analog_pin = 0;

void setup(){
    Serial.begin(9600);
}

void loop(){
    // Retrieve Data
    data = analogRead(analog_pin);

    // Low Pass Filter
    data_filtered[n] = alpha * data + (1 - alpha) * data_filtered[n-1];

    // Store the last filtered data in data_filtered[n-1]
    data_filtered[n-1] = data_filtered[n];
    // Print Data
    Serial.println(data_filtered[n]);

    delay(100);
}
