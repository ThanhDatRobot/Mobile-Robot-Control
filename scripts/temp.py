import matplotlib.pyplot as plt

# Plot the desired and real trajectories in the first window
plt.figure()
plt.plot(self.x_desired, self.y_desired, 'r--', label='Quỹ đạo mong muốn')
plt.plot(self.x_real, self.y_real, 'b-', label='Quỹ đạo thực tế')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Theo dõi đường đi của robot')
plt.legend()
plt.gca().set_aspect('equal', 'box')  # Đảm bảo tỷ lệ chính xác giữa các trục
plt.show()

# Plot the tracking errors in the second window
plt.figure()
plt.plot(self.error_x_list, 'r-', label='Lỗi X')
plt.plot(self.error_y_list, 'b-', label='Lỗi Y')
plt.xlabel('Bước thời gian')
plt.ylabel('Lỗi')
plt.title('Lỗi theo dõi')
plt.legend()
plt.tight_layout()
plt.show()
