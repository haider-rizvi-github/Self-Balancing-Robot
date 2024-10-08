# Pinout
<img src= "https://github.com/haider-rizvi-github/Self-Balancing-Robot/blob/main/reference%20material/Pin%20ports.png"  width="520">

# Motor Drive
To save IO pins we use TB6612FNG (H-bridge) in conjunction with SN74LVC2G14 (Two-Way Schmitt Trigger Reverser) like below 
<img src= "https://github.com/user-attachments/assets/eca88126-7f42-417b-86ec-745b861a1531"  width="480">
<img src= "https://github.com/user-attachments/assets/c2305d4b-a919-4fbb-9a67-0044d94804b9"  width="240">

## Truth Table:
The resultant Truth Table looks like following:

<img src= "https://github.com/user-attachments/assets/2419061f-ebe0-4e08-97cf-f5a7c60095f1"  width="240">

## Flow Chart 
The overall implementation framework of the car motion is as shown below:
<img src= "https://github.com/user-attachments/assets/203b838d-be3a-4089-a271-cb5dfee86af4"  width="480">
