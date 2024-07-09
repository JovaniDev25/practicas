import numpy as np
import matplotlib.pyplot as plt

# Generate a Gaussian distribution with mean = 0 and standard deviation = 1
mu, sigma = 0, 1
s = np.random.normal(mu, sigma, 1000)

# Create a histogram
count, bins, ignored = plt.hist(s, 30, density=True)

# Plot the distribution curve
plt.plot(bins, 1/(sigma * np.sqrt(2 * np.pi)) * np.exp( - (bins - mu)**2 / (2 * sigma**2) ), linewidth=2, color='r')
plt.show()