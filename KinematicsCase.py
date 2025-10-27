import numpy as np
import matplotlib.pyplot as plt
import math

class ForwardKinematics2DOF:
    def __init__(self, femur_length=73, tibia_length=50):
        self.l1, self.l2 = femur_length, tibia_length

        # Applying Homogeneous Transform Matrix H1, H2, H3, and H4 from the PPT Day 2 page 18
    def calculate_position(self, theta1, theta2):
        """rotation matrix for joint 1"""
        H1 = np.array([
            [math.cos(math.radians(theta1)), -math.sin(math.radians(theta1)), 0],
            [math.sin(math.radians(theta1)), math.cos(math.radians(theta1)), 0],
            [0, 0, 1]
        ])

        """Translation matrix for femur length"""
        H2 = np.array([[1, 0, self.l1], [0, 1, 0], [0, 0, 1]])

        """Translation Matrix for joint 2"""
        H3 = np.array([
            [math.cos(math.radians(theta2)), -math.sin(math.radians(theta2)), 0],
            [math.sin(math.radians(theta2)), math.cos(math.radians(theta2)), 0],
            [0, 0, 1]
        ])

        """Translation Matrix for tibia length"""
        H4 = np.array([[1, 0, self.l2], [0, 1, 0], [0, 0, 1]])


        """Calculate End effector position using H4_0 = H1^0 * H2^1 * H3^2 * H4^3"""
        # Combined transformation: Formula already stated above (taken from PPT Day 2 page 18)
        H_total = H1 @ H2 @ H3 @ H4
        return H_total[0, 2], H_total[1, 2], H_total


    def plot_robot_arm(self, theta1, theta2, x_end, y_end,):
        """Plot the robot arm config"""
        #Calculate intermediate points
        x_femur = self.l1 * math.cos(math.radians(theta1))
        y_femur = self.l1 * math.sin(math.radians(theta1))

        plt.figure(figsize=(8,6))
        plt.plot([0, x_femur], [0, y_femur], 'b-', linewidth=3, label=f'Femur ({self.l1}mm)')
        plt.plot([x_femur, x_end], [y_femur, y_end], 'r-', linewidth=3, label=f'Tibia ({self.l2}mm)')
        plt.plot(0, 0, 'ko', markersize=10, label='Base')
        plt.plot(x_femur, y_femur, 'go', markersize=8, label='Knee')
        plt.plot(x_end, y_end, 'ro', markersize=8, label='End Effector')

        plt.xlabel('X(mm)')
        plt.ylabel('Y(mm)')
        plt.title(f'2-DOF Robot Leg\n01={theta1}, 02={theta2} --> End: ({x_end:.1f}, {y_end:.1f})')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')
        plt.show()

class InverseKinematics2DOF:
    def __init__(self, femur_length=73, tibia_length=50):
        self.l1, self.l2 = femur_length, tibia_length

    def calculate_angles(self, x, y):
        """Calculate joint angles for target position"""
        d = math.sqrt(x**2 + y**2)
        if d > (self.l1 + self.l2) or d < abs(self.l1 - self.l2):
            raise ValueError("position not reachable")

        # Elbow up solution
        cos_theta = (x**2 + y**2 - self.l1**2 - self.l2**2)/(2*self.l1*self.l2)
        theta2 = math.degrees(math.acos(max(min(cos_theta, 1), -1)))

        beta = math.atan2(y, x)
        phi = math.acos((self.l1**2 + d**2 - self.l2**2)/(2*self.l1* d))
        theta1 = math.degrees(beta - phi)

        return theta1, theta2

def main():
    fk = ForwardKinematics2DOF(73, 50)
    ik = InverseKinematics2DOF(73, 50)

    while True:
        print("\n1. Forward Kinematics\n2. Inverse Kinematics\n3. Example (40, 30)\n4. Exit")
        choice = input("Enter your choice: ")

        if choice =='1':
            theta1, theta2 = float(input("Theta 1: ")), float(input("Theta 2: "))
            x, y, H = fk.calculate_position(theta1, theta2)
            print(f"End Effector: {x:.2f}, {y:.2f}")
            print("Transformation Matrix:\n", H)
            fk.plot_robot_arm(theta1, theta2, x, y)

        elif choice =='2':
            x, y = float(input("X: ")), float(input("Y: "))
            theta1, theta2 = ik.calculate_angles(x, y)
            print(f"Angles: 01={theta1:.1f}, 02={theta2:.1f}")
            #Verify
            x_verify, y_verify, _ = fk.calculate_position(theta1, theta2)
            print(f"Verification: ({x_verify:.2f}, {y_verify:.2f})")
            fk.plot_robot_arm(theta1, theta2, x_verify, y_verify)

# Angles 40 and 30 are taken from the Study Case
        elif choice == '3':
            x, y, H = fk.calculate_position(40, 30)
            print (f"Example: 01=40, 02=30 = End: ({x:.2f}, {y:.2f})")
            print ("final Matrix:\n", H)
            fk.plot_robot_arm(40, 30, x, y)

        elif choice == '4':
            break

if __name__ == '__main__':
    main()
