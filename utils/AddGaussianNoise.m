function noisy_obstacles = AddGaussianNoise(array, sigma)
noisy_obstacles = sigma * randn(size(array)) + array;
end
