class MyClass
{
private:
  int _arrSize;
  String *_myArr;

public:
  // Constructor
  MyClass(int arrSize) : _arrSize(arrSize)
  {
    _myArr = new String[arrSize];
  }

  // Destructor
  ~MyClass()
  {
    delete []_myArr;
  }

  void addElement(const String *element, int position)
  {
    _myArr[position] = *element;
  }

  String& getElement(int position) const
  {
    return _myArr[position];
  }
};

const String array[2] = {"Obj1", "Obj2"};
MyClass myObject(2);

void setup()
{
  Serial.begin(9600);

  myObject.addElement(&array[0], 0);
  myObject.addElement(&array[1], 1);

  delay(2000);

  Serial.println(myObject.getElement(0));
  Serial.println(myObject.getElement(1));
}

void loop() {}
