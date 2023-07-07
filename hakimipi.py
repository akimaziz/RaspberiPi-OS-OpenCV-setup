import cv2
import numpy as np
import RPi.GPIO as GPIO

cap = cv2.VideoCapture(0)
cap.set(3, 160)
cap.set(4, 120)

in1 = 4
in2 = 17
in3 = 27
in4 = 22
en1 = 23
en2 = 24

GPIO.setwarnings(False)  # Disable GPIO warnings
GPIO.setmode(GPIO.BCM)
GPIO.setup(en1, GPIO.OUT)
GPIO.setup(en2, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
p1 = GPIO.PWM(en1, 150)
p2 = GPIO.PWM(en2, 150)
p1.start(40)
p2.start(40)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)

previous_error = 0

while True:
    ret, frame = cap.read()

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the black line color
    lower_black = np.array([0, 0, 0], dtype=np.uint8)
    upper_black = np.array([180, 255, 50], dtype=np.uint8)

    # Apply color thresholding to isolate the black line
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Apply morphology operations to enhance the black line detection
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        if M["m00"] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            print("CX: " + str(cx) + "  CY: " + str(cy))

            # Calculate the error as the difference between the current x-coordinate and the desired center (80 in this case)
            error = cx - 80

            # Adjust the scaling factor as per your requirements
            proportional_control = 0
            derivative_control = 1
            
            previous_error = 1
            integral_term = 2

            # Calculate the change in error
            delta_error = error - previous_error

            # Calculate the steering command based on the error and proportional control
            steering = int(proportional_control * error + derivative_control * delta_error)

            # Limit the steering command within a certain range
            max_steering = 1
            steering = max(-max_steering, min(steering, max_steering))

            # Set the motor speeds and directions
            if steering < 0:
                print("Turn Right")
                GPIO.output(in1, GPIO.LOW)
                GPIO.output(in2, GPIO.HIGH)
                GPIO.output(in3, GPIO.LOW)
                GPIO.output(in4, GPIO.HIGH)
                p1.ChangeDutyCycle(100)
                p2.ChangeDutyCycle(1 - abs(steering))
            elif steering > 0:
                print("Turn Left")
                GPIO.output(in1, GPIO.HIGH)
                GPIO.output(in2, GPIO.LOW)
                GPIO.output(in3, GPIO.HIGH)
                GPIO.output(in4, GPIO.LOW)
                p1.ChangeDutyCycle(1 - abs(steering))
                p2.ChangeDutyCycle(100)
            else:
                print("On Track!")
                GPIO.output(in1, GPIO.LOW)
                GPIO.output(in2, GPIO.HIGH)
                GPIO.output(in3, GPIO.LOW)
                GPIO.output(in4, GPIO.HIGH)
                p1.ChangeDutyCycle(80)
                p2.ChangeDutyCycle(80)

            # Update the previous error
            previous_error = error

            # Draw a white dot at the center of the line
            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

        else:
            print("I don't see the line")
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.HIGH)
            GPIO.output(in3, GPIO.HIGH)
            GPIO.output(in4, GPIO.HIGH)
            p1.ChangeDutyCycle(0)
            p2.ChangeDutyCycle(0)

        cv2.drawContours(frame, c, -1, (0, 255, 0), 1)

    cv2.imshow("Mask", mask)
    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.HIGH)
        GPIO.output(in3, GPIO.HIGH)
        GPIO.output(in4, GPIO.HIGH)
        p1.stop()
        p2.stop()
        break

cap.release()
cv2.destroyAllWindows()
