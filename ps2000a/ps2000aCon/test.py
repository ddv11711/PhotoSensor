import pandas as pd
import matplotlib.pyplot as plt

# Чтение данных
file_path = 'stream.txt'
data = pd.read_csv(file_path, skiprows=1, delimiter=',')

# Предположим, что значения находятся во втором столбце (индекс 1)
values = data.iloc[:, 1]

# Даунсэмплинг: оставляем каждую 100-ую точку (можно изменить)
downsampled_values = values[::100]

# Преобразование индекса
# Старый индекс: [0, 100, 200, ..., 14000] (после даунсэмплинга)
# Новый индекс: [0, 21.43, 42.86, ..., 3000] миллисекунд
old_index = range(0, len(values), 100)  # Старый индекс после даунсэмплинга
new_index = [(x / 14000) * 3000 for x in old_index]  # Новый индекс в миллисекундах

# Построение графика
plt.plot(new_index, downsampled_values, linestyle='-', color='b')
plt.title('График с уменьшенным количеством точек')
plt.xlabel('Время (мС)')
plt.ylabel('Значение (мВ)')
plt.grid(True)
plt.show()
