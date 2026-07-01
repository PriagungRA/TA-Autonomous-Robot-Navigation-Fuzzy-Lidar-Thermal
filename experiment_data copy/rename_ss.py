import os

# Ganti dengan path folder yang berisi file PNG
folder_path = r"/home/ubuntu/TA/final_ws/experiment_data/S8"

# Ambil semua file PNG
png_files = [f for f in os.listdir(folder_path) if f.lower().endswith(".png")]

# Urutkan berdasarkan nama file lama
png_files.sort()

# Rename satu per satu
for i, old_name in enumerate(png_files, start=1):
    old_path = os.path.join(folder_path, old_name)
    new_name = f"S8_{i}_baseline.png"
    new_path = os.path.join(folder_path, new_name)

    os.rename(old_path, new_path)
    print(f"{old_name} -> {new_name}")

print("Selesai!")