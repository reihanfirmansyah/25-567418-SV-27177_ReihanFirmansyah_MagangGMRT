import math
import numpy as np
import matplotlib.pyplot as plt


def buat_matriks_transformasi_2d(sudut_rad, panjang_link):
    """
    Membuat Homogeneous Transformation Matrix 2D (ukuran 3x3)
    berdasarkan sudut rotasi (radian) dan panjang segmen kaki (cm).
    """
    c = np.cos(sudut_rad)
    s = np.sin(sudut_rad)
    
    matriks = np.array([
        [c, -s, panjang_link * c],
        [s,  c, panjang_link * s],
        [0,  0, 1]
    ])
    return matriks


def hitung_fk_matriks(a1, a2, q1_deg, q2_deg):
    """
    Menghitung Forward Kinematics 2-DoF menggunakan perkalian
    Homogeneous Transformation Matrix (NumPy).
    """
    q1 = math.radians(q1_deg)
    q2 = math.radians(q2_deg)

    # Matriks transformasi masing-masing sendi
    T1 = buat_matriks_transformasi_2d(q1, a1)
    T2 = buat_matriks_transformasi_2d(q2, a2)

    # Perkalian matriks untuk mendapatkan posisi ujung Tibia dari titik pusat (0,0)
    T_total = T1 @ T2

    # Ambil koordinat (x, y) dari kolom ke-3 indeks 2 matriks
    x_femur, y_femur = T1[0, 2], T1[1, 2]
    x_akhir, y_akhir = T_total[0, 2], T_total[1, 2]

    return (x_femur, y_femur), (x_akhir, y_akhir), T_total


def hitung_ik_2dof(a1, a2, target_x, target_y):
    """
    Menghitung Inverse Kinematics (mencari sudut q1 dan q2 dari target X, Y).
    Dilengkapi pengecekan batas jangkauan (workspace).
    """
    jarak_target = math.sqrt(target_x**2 + target_y**2)
    jangkauan_maks = a1 + a2
    jangkauan_min = abs(a1 - a2)

    # Cek apakah target berada di luar jangkauan kaki robot
    if jarak_target > jangkauan_maks or jarak_target < jangkauan_min:
        print("\n[PERINGATAN] Target berada di luar jangkauan (Out of Workspace)!")
        print(f"Jarak target: {jarak_target:.2f} cm | Rentang jangkauan: {jangkauan_min} - {jangkauan_maks} cm")
        return None

    # Aturan Kosinus untuk mencari sudut q2
    cos_q2 = (target_x**2 + target_y**2 - a1**2 - a2**2) / (2 * a1 * a2)
    # Mengamankan nilai cos_q2 dari error pembulatan floating-point di luar [-1, 1]
    cos_q2 = max(-1.0, min(1.0, cos_q2))
    q2_rad = math.acos(cos_q2)

    # Mencari sudut q1
    q1_rad = math.atan2(target_y, target_x) - math.atan2(a2 * math.sin(q2_rad), a1 + a2 * math.cos(q2_rad))

    return math.degrees(q1_rad), math.degrees(q2_rad)


def visualisasi_robot(titik_femur, titik_akhir, q1_deg, q2_deg):
    """
    Menampilkan postur kaki robot menggunakan Matplotlib
    dan menyimpan hasilnya sebagai gambar PNG.
    """
    x_points = [0, titik_femur[0], titik_akhir[0]]
    y_points = [0, titik_femur[1], titik_akhir[1]]

    plt.figure(figsize=(7, 7))
    # Gambar segmen Femur
    plt.plot(x_points[0:2], y_points[0:2], '-o', color='tomato', linewidth=5, markersize=8, label='Femur (18 cm)')
    # Gambar segmen Tibia
    plt.plot(x_points[1:3], y_points[1:3], '-o', color='limegreen', linewidth=5, markersize=8, label='Tibia (77 cm)')
    # Titik Coxa dan End-Effector
    plt.plot(0, 0, 'ko', markersize=10, label='Sendi Coxa (0,0)')
    plt.plot(titik_akhir[0], titik_akhir[1], 'bo', markersize=10, label=f'End-Effector ({titik_akhir[0]:.2f}, {titik_akhir[1]:.2f})')

    plt.title(f"Visualisasi Kaki Robot 2-DoF (q1={q1_deg:.1f}°, q2={q2_deg:.1f}°)")
    plt.xlabel("Sumbu X (cm)")
    plt.ylabel("Sumbu Y (cm)")
    plt.axhline(0, color='gray', linestyle='--', linewidth=0.8)
    plt.axvline(0, color='gray', linestyle='--', linewidth=0.8)
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.axis('equal')
    plt.legend()

    # Simpan otomatis ke file gambar
    nama_file = "visualisasi_postur_robot.png"
    plt.savefig(nama_file, dpi=300)
    print(f"\n[INFO] Gambar grafik berhasil disimpan sebagai '{nama_file}'")
    plt.show()


def main():
    # Panjang segmen berdasarkan 2 digit terakhir NIU saya (567418) dan NIF saya (27177)
    a1 = 18.0  # Femur (cm)
    a2 = 77.0  # Tibia (cm)

    print("SISTEM KINEMATIKA KAKI ROBOT 2-DoF")
    print(f"Femur (a1) = {a1} cm | Tibia (a2) = {a2} cm")
    print("Pilih Mode Perhitungan:")
    print("1. Forward Kinematics (Input Sudut -> Cari Koordinat X, Y)")
    print("2. Inverse Kinematics (Input Target X, Y -> Cari Sudut Servo)")
    pilihan = input("Masukkan pilihan (1/2): ")

    if pilihan == "1":
        q1 = float(input("\nMasukkan sudut Servo 1 / q1 (derajat): "))
        q2 = float(input("Masukkan sudut Servo 2 / q2 (derajat): "))

        titik_femur, titik_akhir, matriks_total = hitung_fk_matriks(a1, a2, q1, q2)

        print(" HASIL FORWARD KINEMATICS ")
        print("Homogeneous Transformation Matrix Total:")
        print(np.round(matriks_total, 4))
        print(f"\nKoordinat Sendi Lutut  : ({titik_femur[0]:.2f}, {titik_femur[1]:.2f}) cm")
        print(f"Koordinat End-Effector : ({titik_akhir[0]:.2f}, {titik_akhir[1]:.2f}) cm")

        visualisasi_robot(titik_femur, titik_akhir, q1, q2)

    elif pilihan == "2":
        tx = float(input("\nMasukkan target koordinat X (cm): "))
        ty = float(input("Masukkan target koordinat Y (cm): "))

        hasil_ik = hitung_ik_2dof(a1, a2, tx, ty)
        if hasil_ik is not None:
            q1_solusi, q2_solusi = hasil_ik
            print("\n--- HASIL INVERSE KINEMATICS ---")
            print(f"Sudut Servo 1 (q1) yang dibutuhkan : {q1_solusi:.2f}°")
            print(f"Sudut Servo 2 (q2) yang dibutuhkan : {q2_solusi:.2f}°")

            # Menampilkan visualisasinya
            titik_femur, titik_akhir, _ = hitung_fk_matriks(a1, a2, q1_solusi, q2_solusi)
            visualisasi_robot(titik_femur, titik_akhir, q1_solusi, q2_solusi)
    else:
        print("Pilihan tidak valid.")


if __name__ == "__main__":
    main()