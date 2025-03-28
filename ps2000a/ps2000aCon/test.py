import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Чтение данных
file_path = 'stream.txt'
data = pd.read_csv(file_path, skiprows=1, delimiter=',')

# Извлекаем значения из 4-го столбца (индекс 3)
values = data.iloc[:, 3]

# Преобразование индекса в миллисекунды (используем ВСЕ точки)
new_index = [(i / 14000) * 3000 for i in range(len(values))]  # Все точки от 0 до 3000 мс

# --- Настройка фиксированного шага для осей ---
# Ось X: строго шаг 100 мс
x_ticks = np.arange(0, 3001, 100)  # Метки: 0, 100, 200, ..., 3000 мс

# Ось Y: фиксированный диапазон 0-500 мВ с шагом 50 мВ
y_ticks = np.arange(0, 5001, 500)    # Метки: 0, 50, 100, ..., 500 мВ

# Построение графика
plt.figure(figsize=(15, 5))  # Увеличенная ширина для плотных меток
plt.plot(new_index, values,   # Используем ВСЕ данные
         linestyle='-', 
         color='blue',
         linewidth=1.0,       # Тонкая линия для плотного графика
         label='Сигнал (мВ)')

# Настройка осей
plt.xticks(x_ticks)          # Строгий шаг 100 мс
plt.yticks(y_ticks)          # Фиксированный шаг 50 мВ

# Подписи и оформление
plt.title('Полный сигнал с шагом 100 мс по оси X', fontsize=14)
plt.xlabel('Время (мс)', fontsize=12)
plt.ylabel('Напряжение (мВ)', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.5)  # Точечная сетка
plt.legend(fontsize=10)

# Оптимизация отображения
plt.xticks(rotation=45)      # Поворот меток X для читаемости
plt.xlim(0, 3000)           # Жесткие границы оси X
plt.ylim(0, 5000)            # Жесткие границы оси Y
plt.tight_layout()          # Автонастройка отступов

plt.show()
