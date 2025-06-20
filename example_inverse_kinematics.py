
from inverse_kinematics import calculate_angles


if __name__ == "__main__":
    while True:
        try:
            x = float(input("Enter x coordinate: "))
            y = float(input("Enter y coordinate: "))
            z = float(input("Enter z coordinate: "))
            
            theta_1, theta_2, theta_3, out_of_reach = calculate_angles(x, y, z)
            
            if out_of_reach:
                print(f"The point is out of reach. Servo angles: Theta 1: {theta_1:.2f}, Theta 2: {theta_2:.2f}, Theta 3: {theta_3:.2f}")
            else:
                print(f"Servo angles: Theta 1: {theta_1:.2f}, Theta 2: {theta_2:.2f}, Theta 3: {theta_3:.2f}")
        except ValueError:
            print("Invalid input. Please enter numeric values for coordinates.")
        except KeyboardInterrupt:
            print("\nExiting the program.")
            break