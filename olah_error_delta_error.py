import pandas as pd
import matplotlib.pyplot as plt

# Membaca file CSV
df = pd.read_csv('/home/pambudi/Yolov8/Data/Data3/General3/data6_15:14_31Jul.csv')

# Membuat grafik Error
plt.figure(figsize=(14, 7))

# Plot Error
plt.plot(df.index, df['Error'], label='Error')

# Plot Delta Error
plt.plot(df.index, df['Delta Error'], label='Delta Error')

# Garis horizontal pada y=0 (Set Point)
plt.axhline(y=0, color='r', linestyle='--', label='Set Point (0)')

# Memberi judul dan label pada grafik
plt.title('Error dan Delta Error terhadap Waktu')
plt.xlabel('Waktu')
plt.ylabel('Nilai Error dan Delta Error')
plt.legend()
plt.grid(True)

# Membatasi rentang nilai error antara -100 hingga 100 (disesuaikan dengan data)
plt.ylim(-100, 100)

# Membatasi rentang nilai waktu (disesuaikan dengan data)
plt.xlim(df.index.min(), df.index.max())

# Menampilkan grafik
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()
