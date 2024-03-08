import serial

class IMU:
     def __init__(self):
          self.ser=serial.Serial('/dev/ttyUSB0', 9600)
          self.count=0
          self.yaw=0.0

     def get_yaw(self):
          # print("Function call")
          if(self.ser.in_waiting>0):
               self.count += 1
               # print(count)
               # Read serial stream

               line = self.ser.readline()
               # print(line)


               # Avoid first n-liines of the serial information

               if self.count> 10:
                    # Strip serial stream of extra characters

                    line = line.rstrip().lstrip()
                    # print("Original: ",line)

                    line = str(line)
                    line = line.strip("'")
                    line = line.strip("b'")
                    # print("Formatted: ",line)
                    values = line.split("\\t")
                    print(values)
                    _,x=values[0].split(": ")
                    print(float(x))
                    self.yaw=float(x)
               
               else:
                    print("Initialization")
               



if __name__ == "__main__":
    r=IMU()
    while True:
      r.get_yaw()
     #  print("Read: ",r.yaw)
     
     


