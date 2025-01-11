import numpy as np
import matplotlib.pyplot as plt

# Without normalization
def without_normalization(x):
    return np.tanh(x)

# With normalization
def with_normalization(x, total_length):
    # Normalize x to [0, 1]
    normalized_x = x / total_length
    # Then apply tanh to the normalized value
    return np.tanh(normalized_x * 4 - 2)  # scale to roughly [-1, 1]

# Create plot
plt.figure(figsize=(10, 6))

# Total length for comparison
total_length = 100

# X values
x = np.linspace(0, total_length, 200)

# Plot both functions
plt.plot(x, without_normalization(x), label='Without Normalization')
plt.plot(x, with_normalization(x, total_length), label='With Normalization')

plt.title('Tanh Function: Normalized vs Unnormalized')
plt.xlabel('X')
plt.ylabel('tanh(x)')
plt.legend()
plt.grid(True)
plt.show()