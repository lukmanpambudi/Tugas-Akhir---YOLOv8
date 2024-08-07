import pandas as pd
import matplotlib.pyplot as plt

# Membaca file CSV
df = pd.read_csv('/home/pambudi/Yolov8/Data/Data/Model/data3_14:44_31Jul.csv')

# Membuat grafik Error
plt.figure(figsize=(14, 7))
plt.plot(df['Confidence Score'], label='Confidence Score')
# plt.plot(df['Timestamp'], df['Delta Error'], label='Delta Error')

plt.axhline(y=0, color='r', linestyle='--', label='Set Point (0)')

# Memberi judul dan label pada grafik
plt.title('Confidence Score terhadap Waktu')
plt.xlabel('Waktu (s)')
plt.ylabel('Nilai Confidence Score')
plt.legend()
plt.grid(True)

# Membatasi rentang nilai error antara -20 hingga 20
plt.ylim(-5, 5)
plt.xlim(1, 774)

# Menampilkan grafik
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()
