import cv2

# simpan titik yang diklik
points = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))

        print(f"Titik {len(points)}: ({x}, {y})")

        # gambar titik
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)

        # tulis nomor titik
        cv2.putText(
            img,
            str(len(points)),
            (x + 10, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            2
        )

        cv2.imshow("Pilih Titik", img)

img = cv2.imread("gambar.png")

cv2.namedWindow("Pilih Titik")
cv2.setMouseCallback("Pilih Titik", mouse_callback)

print("Klik 8 titik pada gambar.")
print("Tekan 'q' jika sudah selesai.")

while True:
    cv2.imshow("Pilih Titik", img)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cv2.destroyAllWindows()

print("\nHasil koordinat:")
for i, p in enumerate(points):
    print(f"P{i+1} = {p}")