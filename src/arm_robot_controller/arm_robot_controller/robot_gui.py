#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import numpy as np
import math as mt
import threading

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.get_logger().info('Robot Kontrol Arayüzü Başladı')

    def publish_joints(self, angles_deg):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        msg.position = [mt.radians(a) for a in angles_deg]
        self.publisher_.publish(msg)

class RobotKinematicsApp:
    def __init__(self, root, ros_node):
        self.ros_node = ros_node  
        self.root = root
        self.root.title("Robot Kinematik (6 Eksen) - ROS 2 Bağlantılı")
        self.root.geometry("1000x650")
 
        style = ttk.Style()
        style.configure("TLabel", font=("Arial", 10))
        style.configure("TButton", font=("Arial", 10, "bold"))

        main_frame = tk.Frame(root)
        main_frame.pack(fill="both", expand=True, padx=20, pady=20)

        self.left_frame = tk.Frame(main_frame, relief="groove", borderwidth=2)
        self.left_frame.pack(side="left", fill="both", expand=True, padx=10)

        self.right_frame = tk.Frame(main_frame, relief="groove", borderwidth=2)
        self.right_frame.pack(side="right", fill="both", expand=True, padx=10)


        # İLERİ KİNEMATİK (FK)
        tk.Label(self.left_frame, text="İleri Kinematik & Kontrol", font=("Arial", 14, "bold")).pack(pady=10)
        tk.Label(self.left_frame, text="Açılar (Derece)", font=("Arial", 11, "bold")).pack()
        
        self.fk_entries = []
        fk_input_frame = tk.Frame(self.left_frame)
        fk_input_frame.pack(pady=10)

        # 6 Adet Açı Girişi
        for i in range(6):
            row = i // 2
            col = (i % 2) * 2
            tk.Label(fk_input_frame, text=f"Joint {i+1}:").grid(row=row, column=col, padx=5, pady=5, sticky="e")
            entry = tk.Entry(fk_input_frame, width=8)
            entry.insert(0, "0") 
            entry.grid(row=row, column=col+1, padx=5, pady=5)
            self.fk_entries.append(entry)

        # 2. Hesapla Butonu
        btn_calc_fk = tk.Button(self.left_frame, text="ROBOTU HAREKET ETTİR (HESAPLA)", command=self.calculate_fk, bg="#4CAF50", fg="white")
        btn_calc_fk.pack(pady=15, ipadx=10)

        # 3. Sonuçlar (Px, Py, Pz)
        tk.Label(self.left_frame, text="Hesaplanan Uç Konumu (TCP)", font=("Arial", 11, "bold")).pack()
        fk_result_frame = tk.Frame(self.left_frame)
        fk_result_frame.pack(pady=10)

        self.fk_results = {}
        for i, label in enumerate(["Px", "Py", "Pz"]):
            tk.Label(fk_result_frame, text=label).grid(row=i, column=0, padx=5, pady=5)
            entry = tk.Entry(fk_result_frame, width=15)
            entry.grid(row=i, column=1, padx=5, pady=5)
            self.fk_results[label] = entry

        tk.Label(self.right_frame, text="Ters Kinematik", font=("Arial", 14, "bold")).pack(pady=10)

        tk.Label(self.right_frame, text="Hedef Konum", font=("Arial", 11, "bold")).pack()
        ik_pos_frame = tk.Frame(self.right_frame)
        ik_pos_frame.pack(pady=5)

        self.ik_pos_entries = {}
        for i, label in enumerate(["Px", "Py", "Pz"]):
            tk.Label(ik_pos_frame, text=label).grid(row=i, column=0, padx=5, pady=2)
            entry = tk.Entry(ik_pos_frame, width=10)
            entry.insert(0, "0" if label != "Pz" else "400") 
            entry.grid(row=i, column=1, padx=5, pady=2)
            self.ik_pos_entries[label] = entry

        tk.Label(self.right_frame, text="Yönelim (Rotation Matrix)", font=("Arial", 11, "bold")).pack(pady=(10,0))
        ik_rot_frame = tk.Frame(self.right_frame)
        ik_rot_frame.pack(pady=5)

        self.ik_rot_entries = [] 
        default_rot = [0,0,1, 0,-1,0, 1,0,0]
        count = 0
        for r in range(3):
            row_entries = []
            for c in range(3):
                lbl_text = f"r{r+1}{c+1}"
                tk.Label(ik_rot_frame, text=lbl_text).grid(row=r, column=c*2, padx=2, pady=2)
                entry = tk.Entry(ik_rot_frame, width=5)
                entry.insert(0, str(default_rot[count]))
                entry.grid(row=r, column=c*2+1, padx=5, pady=2)
                row_entries.append(entry)
                count += 1
            self.ik_rot_entries.append(row_entries)

        btn_calc_ik = tk.Button(self.right_frame, text="TERS KİNEMATİĞİ HESAPLA & GİT", command=self.calculate_ik, bg="#2196F3", fg="white")
        btn_calc_ik.pack(pady=15, ipadx=10)

        tk.Label(self.right_frame, text="Bulunan Açılar", font=("Arial", 11, "bold")).pack()
        ik_result_frame = tk.Frame(self.right_frame)
        ik_result_frame.pack(pady=10)

        self.ik_results = []
        for i in range(6):
            row = i // 2
            col = (i % 2) * 2
            tk.Label(ik_result_frame, text=f"Teta{i+1}:").grid(row=row, column=col, padx=5, pady=5, sticky="e")
            entry = tk.Entry(ik_result_frame, width=8)
            entry.grid(row=row, column=col+1, padx=5, pady=5)
            self.ik_results.append(entry)

    def calculate_fk(self):
        try:

            joints = [float(entry.get()) for entry in self.fk_entries]
            self.ros_node.publish_joints(joints)
            # İleri kinematik 
            t = [mt.radians(a) for a in joints]

            l1 = 145.0  
            l2 = 180.0  
            l3 = 260.0 
            d6 = 130.0 

            T01 = np.array([
                [mt.cos(t[0]), -mt.sin(t[0]), 0, 0],
                [mt.sin(t[0]),  mt.cos(t[0]), 0, 0],
                [0,               0,              1, l1],
                [0,               0,              0, 1]
            ])

            T12 = np.array([
                [mt.cos(t[1]), -mt.sin(t[1]), 0, 0],
                [0,               0,             -1, 0],
                [mt.sin(t[1]),  mt.cos(t[1]), 0, 0],
                [0,               0,              0, 1]
            ])

            angle23 = t[2] + mt.pi/2
            T23 = np.array([
                [mt.cos(angle23), -mt.sin(angle23), 0, l2],
                [mt.sin(angle23),  mt.cos(angle23), 0, 0],
                [0,                  0,                 1, 0],
                [0,                  0,                 0, 1]
            ])

            T34 = np.array([
                [mt.cos(t[3]), -mt.sin(t[3]),  0,   0],
                [0,               0,              -1, -l3],
                [mt.sin(t[3]),  mt.cos(t[3]),  0,   0],
                [0,               0,               0,   1]
            ])

            T45 = np.array([
                [mt.cos(t[4]), -mt.sin(t[4]),  0, 0],
                [0,               0,              -1, 0],
                [mt.sin(t[4]),  mt.cos(t[4]),  0, 0],
                [0,               0,               0, 1]
            ])
        
            T56 = np.array([
                [mt.cos(t[5]), -mt.sin(t[5]), 0,  0],
                [0,               0,              1, d6],
                [-mt.sin(t[5]), -mt.cos(t[5]), 0,  0],
                [0,               0,              0,  1]
            ])

            T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56

       
            px_val = T06[0, 3]
            py_val = T06[1, 3]
            pz_val = T06[2, 3]

            self.fk_results["Px"].delete(0, tk.END)
            self.fk_results["Py"].delete(0, tk.END)
            self.fk_results["Pz"].delete(0, tk.END)
            
            self.fk_results["Px"].insert(0, f"{px_val:.2f}")
            self.fk_results["Py"].insert(0, f"{py_val:.2f}")
            self.fk_results["Pz"].insert(0, f"{pz_val:.2f}")

        except ValueError:
            messagebox.showerror("Hata", "Lütfen açılar için geçerli sayısal değerler girin.")

    def calculate_ik(self):
        try:
            pxs = float(self.ik_pos_entries["Px"].get())
            pys = float(self.ik_pos_entries["Py"].get())
            pzs = float(self.ik_pos_entries["Pz"].get())

            R06 = np.zeros((3,3))
            for r in range(3):
                for c in range(3):
                    R06[r, c] = float(self.ik_rot_entries[r][c].get())

            l1 = 145.0
            l2 = 180.0
            l3 = 260.0
            d6 = 130.0
            Px = pxs - d6 * R06[0, 2]
            Py = pys - d6 * R06[1, 2]
            Pz = pzs - d6 * R06[2, 2]

            t1 = mt.atan2(Py, Px)

            c = Px * mt.cos(t1) + Py * mt.sin(t1)
            d = Pz - l1

            d1_val = (c**2 + d**2 - l3**2 - l2**2) / (2 * l3 * l2)
            d1_val = max(min(d1_val, 1.0), -1.0) 

            d2_val = mt.sqrt(1 - d1_val**2)

            t3 = mt.atan2(d2_val, d1_val)

            r = l3 * d1_val + l2
            s = l3 * d2_val

            t2 = mt.atan2(r * d - s * c, r * c + s * d)
            T01 = np.array([
                [mt.cos(t1), -mt.sin(t1), 0, 0],
                [mt.sin(t1),  mt.cos(t1), 0, 0],
                [0,             0,            1, l1],
                [0,             0,            0, 1]
            ])

            T12 = np.array([
                [mt.cos(t2), -mt.sin(t2), 0, 0],
                [0,             0,           -1, 0],
                [mt.sin(t2),  mt.cos(t2), 0, 0],
                [0,             0,            0, 1]
            ])
            angle23 = t3 + mt.pi/2
            T23 = np.array([
                [mt.cos(angle23), -mt.sin(angle23), 0, l2],
                [mt.sin(angle23),  mt.cos(angle23), 0, 0],
                [0,                  0,                 1, 0],
                [0,                  0,                 0, 1]
            ])

            T34 = np.array([
                [1, 0,  0,   0],
                [0, 0, -1, -l3],
                [0, 1,  0,   0],
                [0, 0,  0,   1]
            ])
            
            T04 = T01 @ T12 @ T23 @ T34
            
            R04 = T04[0:3, 0:3]
            R64 = R04.T @ R06

            t5 = mt.atan2(mt.sqrt(1 - R64[2, 2]**2), R64[2, 2])
            t4 = mt.atan2(-R64[1, 2], -R64[0, 2])
            t6 = mt.atan2(-R64[2, 1], R64[2, 0])  

            thetas_rad = [t1, t2, t3, t4, t5, t6]
            thetas_deg = [mt.degrees(t) for t in thetas_rad]
            
            for i, entry in enumerate(self.ik_results):
                entry.delete(0, tk.END)
                entry.insert(0, f"{thetas_deg[i]:.2f}")

            self.ros_node.publish_joints(thetas_deg)

        except Exception as e:
            messagebox.showerror("Hata", f"Hesaplama Hatası: {str(e)}")
            print(f"Detaylı Hata: {e}")

def main(args=None):
    rclpy.init(args=args)
    ros_node = RobotControlNode()
    thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    thread.start()

    root = tk.Tk()
    app = RobotKinematicsApp(root, ros_node) 
    root.mainloop()

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()