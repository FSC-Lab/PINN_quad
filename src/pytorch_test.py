import torch

print(torch.__version__)
print(torch.cuda.is_available())
print(torch.version.cuda)
print(torch.backends.cudnn.version())
print(torch.cuda.device_count())

print("Checking PyTorch GPU availability...\n")

print("CUDA Available:", torch.cuda.is_available())  # Should print True
print("GPU Count:", torch.cuda.device_count())  # Should be 1
print("GPU Name:", torch.cuda.get_device_name(0))  # Should print "GeForce GTX 960M"
print("CUDA Version:", torch.version.cuda)  # Should print 12.1
print("PyTorch Version:", torch.__version__)  # Should print 2.4.1+cu121


x = torch.rand(10000, 10000).cuda()  # Moves a large tensor to GPU
y = torch.mm(x, x)  # Matrix multiplication on GPU
print("Computation successful!")


