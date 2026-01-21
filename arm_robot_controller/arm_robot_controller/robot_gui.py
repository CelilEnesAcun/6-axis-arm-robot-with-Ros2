#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import numpy as np
import math
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
        msg.position = [math.radians(a) for a in angles_deg]
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

        # ==========================================
        # SOL TARAF: İLERİ KİNEMATİK (FK)
        # ==========================================
        tk.Label(self.left_frame, text="İleri Kinematik & Kontrol", font=("Arial", 14, "bold")).pack(pady=10)
        
        # 1. Açılar (Inputs)
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
            entry.insert(0, "0") # Varsayılan değer
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

        # ==========================================
        # SAĞ TARAF: TERS KİNEMATİK (IK)
        # ==========================================
        tk.Label(self.right_frame, text="Ters Kinematik", font=("Arial", 14, "bold")).pack(pady=10)

        # 1. Konum Girdileri (Px, Py, Pz)
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

        # 2. Yönelim Matrisi Girdileri
        tk.Label(self.right_frame, text="Yönelim (Rotation Matrix)", font=("Arial", 11, "bold")).pack(pady=(10,0))
        ik_rot_frame = tk.Frame(self.right_frame)
        ik_rot_frame.pack(pady=5)

        self.ik_rot_entries = [] 
        default_rot = [1,0,0, 0,1,0, 0,0,1]
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

        # 3. Hesapla Butonu
        btn_calc_ik = tk.Button(self.right_frame, text="TERS KİNEMATİĞİ HESAPLA & GİT", command=self.calculate_ik, bg="#2196F3", fg="white")
        btn_calc_ik.pack(pady=15, ipadx=10)

        # 4. Sonuçlar (Açılar)
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
            t1_deg = float(self.fk_entries[0].get())
            t2_deg = float(self.fk_entries[1].get())
            t3_deg = float(self.fk_entries[2].get())
            t4_deg = float(self.fk_entries[3].get())
            t5_deg = float(self.fk_entries[4].get())
            t6_deg = float(self.fk_entries[5].get()) 


            angles_to_send = [t1_deg, t2_deg, t3_deg, t4_deg, t5_deg, t6_deg]
            self.ros_node.publish_joints(angles_to_send)

            theta1 = math.radians(t1_deg)
            theta2 = math.radians(t2_deg)
            theta3 = math.radians(t3_deg)
            theta4 = math.radians(t4_deg)
            theta5 = math.radians(t5_deg)
 
            l1 = 145.0  
            l2 = 180.0  
            l3 = 260.0 
            d6 = 130.0 

            px_val = (
                l3 * (math.cos(theta1) * math.cos(theta2) * math.sin(theta3 + math.pi/2) + 
                      math.cos(theta1) * math.cos(theta3 + math.pi/2) * math.sin(theta2)) 
                - d6 * (
                    math.sin(theta5) * (
                        math.sin(theta1) * math.sin(theta4) - math.cos(theta4) * (
                            math.cos(theta1) * math.sin(theta2) * math.sin(theta3 + math.pi/2) - 
                            math.cos(theta1) * math.cos(theta2) * math.cos(theta3 + math.pi/2)
                        )
                    ) - math.cos(theta5) * (
                        math.cos(theta1) * math.cos(theta2) * math.sin(theta3 + math.pi/2) + 
                        math.cos(theta1) * math.cos(theta3 + math.pi/2) * math.sin(theta2)
                    )
                ) 
                + l2 * math.cos(theta1) * math.cos(theta2)
            )

            py_val = (
                l3 * math.cos(theta2 + theta3) * math.sin(theta1) + 
                l2 * math.cos(theta2) * math.sin(theta1) + 
                d6 * math.cos(theta2 + theta3) * math.cos(theta5) * math.sin(theta1) + 
                d6 * math.cos(theta1) * math.sin(theta4) * math.sin(theta5) + 
                d6 * math.cos(theta2) * math.cos(theta4) * math.sin(theta1) * math.sin(theta3) * math.sin(theta5) + 
                d6 * math.cos(theta3) * math.cos(theta4) * math.sin(theta1) * math.sin(theta2) * math.sin(theta5)
            )

            pz_val = (
                l1 + 
                l2 * math.sin(theta2) + 
                l3 * math.cos(theta2) * math.sin(theta3) + 
                l3 * math.cos(theta3) * math.sin(theta2) + 
                d6 * math.cos(theta2) * math.cos(theta5) * math.sin(theta3) + 
                d6 * math.cos(theta3) * math.cos(theta5) * math.sin(theta2) - 
                d6 * math.cos(theta2) * math.cos(theta3) * math.cos(theta4) * math.sin(theta5) + 
                d6 * math.cos(theta4) * math.sin(theta2) * math.sin(theta3) * math.sin(theta5)
            )

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
            # 1. Konum verilerini al
            px = float(self.ik_pos_entries["Px"].get())
            py = float(self.ik_pos_entries["Py"].get())
            pz = float(self.ik_pos_entries["Pz"].get())

            # 2. Rotasyon Matrisini al (3x3)
            R = np.zeros((3,3))
            for r in range(3):
                for c in range(3):
                    R[r, c] = float(self.ik_rot_entries[r][c].get())

            # -----------------------------------------------------------------
            # BURAYA SENİN YAZACAĞIN TERS KİNEMATİK MATEMATİĞİ GELECEK
            # -----------------------------------------------------------------
            # Örnek: Matematiksel fonksiyonunu çağırıp t1..t6 hesaplayacaksın.
            # Şimdilik örnek olsun diye hepsine 10 derece veriyorum.
            
            calc_thetas = [10, 20, 30, 40, 50, 60]  # <-- BU LİSTE HESAPLANACAK
            
            # -----------------------------------------------------
            # ROS 2'YE GÖNDER (Robot Bulunan Konuma Gitsin)
            # -----------------------------------------------------
            self.ros_node.publish_joints(calc_thetas)
            
            for i, entry in enumerate(self.ik_results):
                entry.delete(0, tk.END)
                entry.insert(0, f"{calc_thetas[i]:.2f}")

        except ValueError:
             messagebox.showerror("Hata", "Lütfen tüm matris ve konum değerlerini sayı olarak girin.")

def run_ros_node(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    

    ros_node = RobotControlNode()
    
    thread = threading.Thread(target=run_ros_node, args=(ros_node,), daemon=True)
    thread.start()

    root = tk.Tk()
    app = RobotKinematicsApp(root, ros_node) 
    root.mainloop()

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()