import serial
import time
import re

def make_phone_call(phone_number) -> bool:

    # Configure the serial connection to the SiM800L
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

    # Give the SiM800L some time to initialize
    time.sleep(2)


    # Send AT command to ensure SiM800L is responsive
    ser.write(b'AT\r')
    response = ser.read(100).decode()
    if 'OK' not in response:
        print("SIM808 not responding. Check the connection.")
        return False

    # Dial the number
    ser.write(f'ATD{phone_number};\r'.encode())
    print(f"Dialing {phone_number}...")

    # Wait a bit for the call to be initiated
    time.sleep(5)

    # Check call status if needed (e.g., 3 seconds after dialing)
    # Optionally, read back module response if needed
    response = ser.read(100).decode()
    print(response)

    # End the call after 10 seconds (or whenever you want to hang up)
    time.sleep(10)
    ser.write(b'ATH\r')
    print("Call ended.")

    # Close the serial connection
    ser.close()
    return True




def read_sms():

    # Set up serial connection to SiM800L
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    # Replace with your serial port


    # Wait for the module to initialize

    time.sleep(2)
    
    
    # Check connection by sending 'AT' command
    ser.write(b'AT\r')
    response = ser.read(100).decode()
    if 'OK' not in response:
        print("SIM808 not responding. Check the connection.")
        return False

    # Set the module to text mode for SMS
    ser.write(b'AT+CMGF=1\r')
    time.sleep(1)
    response = ser.read(100).decode()
    if 'OK' not in response:
        print("Failed to set SMS text mode.")
        return False
    
    
    # Read all SMS messages
    ser.write(b'AT+CMGL="ALL"\r')
    time.sleep(2)
    messages = ser.read(1000).decode()
    
    
    # process and print messages
    if '+CMGL' in messages:
        sms_list = messages.split('+CMGL: ')
        for sms in sms_list[1:]:
            # parse message data

            parts = sms.split('\r\n', 1)
            header = parts[0]
            body = parts[1] if len(parts) > 1 else ""

            # Use regex to capture the sender and date from header
            match = re.search(r'(\d+),”(.*?)”,”(.*?)”,(.*?),”(.+?)”', header)
            if match:
                sender = match.group(3)
                date = match.group(5)
                print(f"Message from: {sender}")
                print(f"Received on: {date}")
                print("Message:")
                print(body)
                print("-" * 40)
            else:
                print("No SMS messages found.")

    # Close the serial connection
    ser.close()