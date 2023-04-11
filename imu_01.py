import serial
ser = serial.Serial('/dev/ttyUSB0', 9600)

count=0

while True:
     
     if(ser.in_waiting>0):
          count += 1
          print(count)
          # Read serial stream

          line = ser.readline()
          print(line)


          # Avoid first n-liines of the serial information

          if count> 100:
               # Strip serial stream of extra characters

               line = line.rstrip().lstrip()
               print(line)

               line = str(line)
               line = line.strip("'")
               line = line.strip("b'")
               print(line)


               #Return float
            #    print(line)
               line = float(line)

               print(line,"\n")