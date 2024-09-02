import asyncio
import math

from mavsdk import System
from mavsdk import telemetry
import time
import serial

# Function to rotate LiDAR servo

ser = serial.Serial(
	port='/dev/ttyUSB0',
	baudrate=115200,
	timeout=1
)

async def rotate_lidar_servo(angle, latitude, longitude, altitude, drone):
    # Implement the function to rotate your servo to the specified angle
    # For example, using GPIO pins on a Raspberry Pi
    await drone.action.goto_location(latitude, longitude, altitude, angle)

# Function to get distance from LiDAR sensor

def get_lidar_distance():
    while True:
        if ser.in_waiting >= 9:  # Minimum number of bytes for a valid frame
            data = ser.read(9)
            if data[0] == 0x59 and data[1] == 0x59:  # Frame header
                distance = data[2] + data[3] * 256
                strength = data[4] + data[5] * 256
                temperature = data[6] + data[7] * 256
                temperature = (temperature / 8.0) - 256.0  # Convert temperature to Celsius

                print(f"Distance: {distance} cm, Strength: {strength}, Temperature: {temperature:.2f} °C")

            ser.reset_input_buffer()
        time.sleep(0.1)
    return distance

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14500")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(5)

    async for position in drone.telemetry.position():
        latitude = position.latitude_deg
        longitude = position.longitude_deg
        altitude = position.absolute_altitude_m
        print(f"Latitude: {latitude}, Longitude: {longitude}")
        # Add a break or condition if you don't want to run indefinitely
        break

    latitudet = latitude + .0000318
    longitudet = longitude + .0000449
    altitudet = altitude
    not_at_target = True

    while not_at_target:
        distances = []
        angle = math.atan((latitudet-latitude)/(longitudet-longitude))
        await rotate_lidar_servo(angle, latitude, longitude, altitude, drone)
        distance = get_lidar_distance()
        if distance < 500:
            for angle in [0, 90, 180, 270]:
                await rotate_lidar_servo(angle, latitude, longitude, altitude, drone)
                time.sleep(1)  # Allow time for the servo to rotate and stabilize
                distance = get_lidar_distance()
                distances.append((angle, distance))
                print(f"Angle: {angle} degrees, Distance: {distance} cm")

            # Find the direction with the most distance (least obstacle)
            best_angle, max_distance = max(distances, key=lambda x: x[1])
            yaw = angle
            if max_distance <= 400:  # Threshold distance in cm to avoid obstacles
                print(f"Obstacle too close at all angles. Hovering.")
                await drone.action.hold()
            elif max_distance > 400:
                print(f"Moving in direction: {best_angle} degrees")
                if best_angle == 0:
                    await drone.action.goto_location(latitude, longitude + 0.0000449, altitude, yaw)
                elif best_angle == 90:
                    await drone.action.goto_location(latitude + 0.0000318, longitude, altitude, yaw)
                elif best_angle == 180:
                    await drone.action.goto_location(latitude, longitude - 0.0000449, altitude, yaw)
                elif best_angle == 270:
                    await drone.action.goto_location(latitude - 0.0000318, longitude, altitude, yaw)

                async for position in drone.telemetry.position():
                    latitude = position.latitude_deg
                    longitude = position.longitude_deg
                    altitude = position.absolute_altitude_m
                    print(f"Latitude: {latitude}, Longitude: {longitude}")
                    # Add a break or condition if you don't want to run indefinitely
                    break
                else:
                    await drone.action.goto_location(latitudet, longitudet, altitude, angle)

                async for position in drone.telemetry.position():
                    latitude = position.latitude_deg
                    longitude = position.longitude_deg
                    altitude = position.absolute_altitude_m
                    print(f"Latitude: {latitude}, Longitude: {longitude}")
                    # Add a break or condition if you don't want to run indefinitely
                    break
                    if ((latitudet-latitude)*(latitudet-latitude) + (longitudet-longitude)*(longitudet-longitude) < .000001):
                        not_at_target = False
                        
            await asyncio.sleep(1)

    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run())

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.