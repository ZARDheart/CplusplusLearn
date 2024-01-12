#include "kernels.cuh"
#include <iostream>
#include <ctime> //时间库，计时用

// 在CPU上运行的函数
void CPUFunction()
{
  printf("This function is defined to run on the CPU.\n");
}

// 在GPU上运行的函数
// __global__ 关键字表明该函数将在GPU上运行并可全局调用（既可以由CPU，也可以由GPU调用）
__global__ void GPUFunction()
{
  printf("This function is defined to run on the GPU.\n");
}

void printDeviceInfo();

// 核函数
__global__ void kernelIndex()
{
  // 当执行到第255个线程块的第1023个线程时，才输出
  if (threadIdx.x == 1023 && blockIdx.x == 255)
  {
    printf("GPU for!\n");                     // 输出 Success！
    printf("threadIdx.x: %d\n", threadIdx.x); // 输出线程ID
    printf("blockIdx.x: %d\n", blockIdx.x);   // 输出线程块ID
    // 沉睡00.1ms
    clock_t start = clock();
    while (clock() - start < CLOCKS_PER_SEC / 100000)
      ;
  }
}

// 相同功能的CPU函数，说明并行的好处
void kernelIndexCPU()
{
  for (int i = 0; i < 256; i++)
  {
    for (int j = 0; j < 1024; j++)
    {
      // 当执行到第255个线程块的第1023个线程时，才输出
      if (j == 1023 && i == 255)
      {
        printf("CPU for!\n");           // 输出 Success！
        printf("threadIdx.x: %d\n", j); // 输出线程ID
        printf("blockIdx.x: %d\n", i);  // 输出线程块ID
      }
      // 沉睡00.1ms
      clock_t start = clock();
      while (clock() - start < CLOCKS_PER_SEC / 100000)
        ;
    }
  }
}

__global__ void loop()
{
  // 在Grid中遍历所有thread
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  printf("%d ", i);
}

// 处理工作量小于线程数的情况
__global__ void some_kernel(int N)
{
  int idx = threadIdx.x + blockIdx.x * blockDim.x;

  if (idx < N) // 保证线程ID小于元素数量N
  {
    printf("%d ", idx + 100);
  }
}

// CUDA 核函数，所有元素乘2
__global__ void doubleElements(int *a, int N)
{
  int i;
  i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < N)
  {
    a[i] *= 2;
  }
}

// 检查数组内所有元素的值是否均为偶数
bool checkElementsAreDoubled(int *a, int N)
{
  int i;
  for (i = 0; i < N; ++i)
  {
    if (a[i] != i * 2)
      return false;
  }
  return true;
}

// 使用grid-stride循环，这样每个线程可以处理数组中的多个元素。
__global__ void doubleElements2(int *a, int N)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = gridDim.x * blockDim.x; // grid 的一个跨步,跳到下一个网格计算

  for (int i = idx; i < N; i += stride)
  {
    a[i] *= 2;
  }
}

// GPU 矩阵乘法
__global__ void matrixMulGPU(int *a, int *b, int *c, int N)
{
  int val = 0;

  int row = blockIdx.y * blockDim.y + threadIdx.y;
  int col = blockIdx.x * blockDim.x + threadIdx.x;

  if (row < N && col < N)
  {
    for (int k = 0; k < N; ++k)
      val += a[row * N + k] * b[k * N + col];
    c[row * N + col] = val;
  }
}

// CPU矩阵乘法
void matrixMulCPU(int *a, int *b, int *c, int N)
{
  int val = 0;

  for (int row = 0; row < N; ++row)
    for (int col = 0; col < N; ++col)
    {
      val = 0;
      for (int k = 0; k < N; ++k)
        val += a[row * N + k] * b[k * N + col];
      c[row * N + col] = val;
    }
}

void dim3grid()
{
  int N = 64;
  int *a, *b, *c_cpu, *c_gpu;
  int size = N * N * sizeof(int); // Number of bytes of an N x N matrix
  // 分配内存
  // 数据
  cudaMallocManaged(&a, size);
  cudaMallocManaged(&b, size);
  // 结果
  cudaMallocManaged(&c_cpu, size);
  cudaMallocManaged(&c_gpu, size);

  // 初始化数组（用一维数组来表示矩阵）
  for (int row = 0; row < N; ++row)
    for (int col = 0; col < N; ++col)
    {
      a[row * N + col] = row;
      b[row * N + col] = col + 2;
      c_cpu[row * N + col] = 0;
      c_gpu[row * N + col] = 0;
    }

  dim3 threads_per_block(16, 16, 1); // 一个 16 * 16 的线程阵
  dim3 number_of_blocks((N / threads_per_block.x) + 1, (N / threads_per_block.y) + 1, 1);
  matrixMulGPU<<<number_of_blocks, threads_per_block>>>(a, b, c_gpu, N); // 执行核函数 GPU矩阵乘法
  cudaDeviceSynchronize();
  matrixMulCPU(a, b, c_cpu, N); // 执行 CPU 版本的矩阵乘法

  // 比较 CPU 和 GPU 两种方法的计算结果是否一致
  bool error = false;
  for (int row = 0; row < N && !error; ++row)
    for (int col = 0; col < N && !error; ++col)
      if (c_cpu[row * N + col] != c_gpu[row * N + col])
      {
        printf("FOUND ERROR at c[%d][%d]\n", row, col);
        error = true;
        break;
      }
  if (!error)
    printf("Success!\n");

  // 释放内存
  cudaFree(a);
  cudaFree(b);
  cudaFree(c_cpu);
  cudaFree(c_gpu);
}

void CppCUDAUsage()
{
  CPUFunction(); // 调用CPU函数

  /* 1 调用GPU函数,通常把要运行在GPU上的函数称为kernel（核）函数
   * 启动核(kernel)函数时，我们必须使用 <<< ... >>> 语法向核函数传递两个必要的参数
   * CUDA线程的层次结构分为三层：Thread（线程）、Block（块）、Grid（网格），网格由块组成，块由线程组成
   * 使用 <<<...>>> 中第一个1表示1个线程块，第二个1表示每个线程块1个线程。
   */
  // 通过这个函数中的内容获取设备信息
  printDeviceInfo();
  GPUFunction<<<2, 2>>>(); // 在GPU中为核函数分配5个具有5个线程的线程块，将运行2*2=4次,且四次同时运算

  // 2 我们将在 CPU 上执行的代码称为 Host （主机）代码，而将在 GPU 上运行的代码称为 Device （设备）代码
  // 与其他并行化的代码类似，核函数启动方式为异步，即CPU代码将继续执行而不会等待核函数执行完成
  // 调用CUDA提供的函数cudaDeviceSynchronize可以让Host代码(CPU)等待Device代码(GPU)执行完毕，再在CPU上继续执行
  cudaDeviceSynchronize();

  // 3 CUDA 核函数在由一个或多个线程块组成的网格中执行，
  // 且每个线程块中均包含相同数量的一个或多个线程（每个线程块中的线程数量相同）
  // 在核函数中，可以通过两个变量来获取到索引： threadIdx.x （线程索引）和 blockIdx.x（线程块索引）
  clock_t start = clock(); // 获得当前时间
  kernelIndex<<<480, 1024>>>();
  cudaDeviceSynchronize();
  clock_t timeGPU = clock();
  double pt = ((double)(timeGPU - start)) / CLOCKS_PER_SEC; // 当前时间-开始标记时间，转化为秒
  std::cout << "GPU run times " << pt << "s" << std::endl;

  // 4 用CUDA加速For循环，同样功能的代码看出来，GPU运算速度差距
  kernelIndexCPU();
  clock_t timeCPU = clock();
  pt = ((double)(timeCPU - timeGPU)) / CLOCKS_PER_SEC;
  std::cout << "CPU run times " << pt << "s" << std::endl;

  // 5 核函数中记录了每个块中线程数的变量是blockDim.x（一个线程块中包含的线程数量，每个块中包含的线程数都是一样的）。
  // 通过将此变量与blockIdx.x和threadIdx.x变量结合使用，并借助表达式threadIdx.x+blockIdx.x*blockDim.x计算线程ID
  loop<<<2, 5>>>();
  cudaDeviceSynchronize();
  printf("\n");

  // 6 鉴于 GPU 的硬件特性，线程块中的线程数最好配置为 32 的倍数，但是在实际工作中，
  // 不可能每次配置参数的时候都手动去算一遍最佳配置，更何况并不是所有的数都是 32 的倍数，通过以下三个步骤轻松地解决：
  //   （1）配置参数，使线程总数超过实际工作所需的线程数
  //   （2）然后，在向核函数传递参数时传递一个用于表示要处理的数据集总大小或完成工作所需的总线程数 N
  //   （3）最后，计算网格内的线程索引后（使用 threadIdx + blockIdx*blockDim），判断该索引是否超过 N，
  // 只在不超过的情况下执行与核函数相关的工作。
  // 假设N是已知的
  // int N = 100000;
  int N = 10;
  // 把每个block中的thread数设为256
  size_t threads_per_block = 256;
  // 根据N和thread数量配置Block数量
  size_t number_of_blocks = (N + threads_per_block - 1) / threads_per_block;
  some_kernel<<<number_of_blocks, threads_per_block>>>(N);
  cudaDeviceSynchronize();
  printf("\n");

  // 7 分配可同时被GPU和CPU访问的内存
  // CUDA可以便捷地分配和释放既可用于 Host 也可被 Device 访问的内存。
  // 在 Host（CPU）中，我们一般适用malloc 和 free 来分配和释放内存，
  // 但这样分配的内存无法直接被Device（GPU）访问，所以在这里我们用cudaMallocManaged 和 cudaFree
  // 两个函数来分配和释放同时可被 Host 和 Device 访问的内存。
  N = 1000;
  int *a;
  size_t size = N * sizeof(int);
  // cudaMallocManaged在统一内存中创建了一个托管内存池（CPU上有，GPU上也有），
  // 内存池中已分配的空间可以通过相同的指针直接被CPU和GPU访问，底层系统在统一的内存空间中自动地在设备和主机间进行传输。
  cudaMallocManaged(&a, size); // 为a分配CPU和GPU空间
  for (int i = 0; i < N; ++i)
    a[i] = i;
  number_of_blocks = (N + threads_per_block - 1) / threads_per_block; // block的数量
  doubleElements<<<number_of_blocks, threads_per_block>>>(a, N);      // 执行核函数，GPU访问
  cudaDeviceSynchronize();
  bool areDoubled = checkElementsAreDoubled(a, N); // 检查元素是否为复数，CPU访问
  printf("All elements were doubled? %s\n", areDoubled ? "TRUE" : "FALSE");
  cudaFree(a); // 释放由cudaMallocManaged

  // 8 反之，如果一个网格中的线程数量可能会小于实际工作量的大小，在核函数中使用跨网格循环
  // CUDA 提供一个记录了网格中线程块数的变量：gridDim.x。然后可以利用它来计算网格中的总线程数，
  // 即网格中的线程块数乘以每个线程块中的线程数：gridDim.x * blockDim.x
  N = 100000;
  int *b;
  cudaMallocManaged(&b, N * sizeof(int));
  for (int i = 0; i < N; ++i)
    b[i] = i;
  // 与上面的那个函数不同，32*256<N, 在一个线程里面循环，算不到的就跳到下一个网格计算
  doubleElements2<<<32, 256>>>(b, N);
  cudaDeviceSynchronize();
  // TRUE说明每个元素都有运行到
  printf("All elements were doubled? %s\n", checkElementsAreDoubled(b, N) ? "TRUE" : "FALSE");

  // 9 CUDA 函数发生错误时会返回一个类型为 cudaError_t 的变量，该变量可用于检查调用函数时是否发生错误。
  cudaError_t syncErr, asyncErr; // 定义错误处理变量
  // 单块线程数大于1024（前面说过每个block的线程数不能超过1024）
  doubleElements2<<<32, 2048>>>(b, N); // 执行核函数
  syncErr = cudaGetLastError();        // cudaGetLastError函数可以用于捕获核函数执行期间发生的错误
  // 捕获同步期间发生的错误，检查后续同步 CPU 与 GPU 时 API 调用所返回的状态（例如 cudaDeviceSynchronize）
  // 如果之前执行的某一个核函数失败了，则将会发生错误
  asyncErr = cudaDeviceSynchronize();
  // 输出错误 说明：两个错误需分别设置（即每次运行时只保留一个错误）
  if (syncErr != cudaSuccess)
    printf("Error: %s\n", cudaGetErrorString(syncErr));
  if (asyncErr != cudaSuccess)
    printf("Error: %s\n", cudaGetErrorString(asyncErr));
  // 输出Error: invalid configuration argument
  cudaFree(b);

  // 10 网格和线程块最多可以定义有 3 个维度，使用多个维度定义网格和线程块在处理具有多个维度的数据时可能很有效，例如二维矩阵。
  // 如果要定义二维或三维的网格或线程块，可以使用 CUDA 的 dim3 关键字来定义多维网格或块
  dim3grid();

  return;
}

void printDeviceInfo()
{
  int device_id = 0; // 选择第一个设备
  cudaDeviceProp device_props;
  cudaGetDeviceProperties(&device_props, device_id);

  // 查询设备的线程块和线程的维度限制
  int max_threads_per_block = device_props.maxThreadsPerBlock;
  int max_threads_per_multiprocessor = device_props.maxThreadsPerMultiProcessor;
  int max_blocks_per_multiprocessor = device_props.maxBlocksPerMultiProcessor;
  dim3 max_threads_dim = {device_props.maxThreadsDim[0], device_props.maxThreadsDim[1], device_props.maxThreadsDim[2]};
  dim3 max_grid_size = {device_props.maxGridSize[0], device_props.maxGridSize[1], device_props.maxGridSize[2]};
  printf("Device %d properties:\n", device_id);
  printf("  Max threads per block: %d\n", max_threads_per_block);
  printf("  Max threads per multiprocessor: %d\n", max_threads_per_multiprocessor);
  printf("  Max blocks per multiprocessor: %d\n", max_blocks_per_multiprocessor);
  printf("  Max thread dimensions: (%d, %d, %d)\n", max_threads_dim.x, max_threads_dim.y, max_threads_dim.z);
  printf("  Max grid size: (%d, %d, %d)\n", max_grid_size.x, max_grid_size.y, max_grid_size.z);
  // Device 0 properties:
  //   Max threads per block: 1024
  //   Max threads per multiprocessor: 1536
  //   Max blocks per multiprocessor: 16
  //   Max thread dimensions: (1024, 1024, 64)
  //   Max grid size: (2147483647, 65535, 65535)
  // 通过上述信息可以看到当前设备（3060）一个线程块最大能够使用的线程数量为1024,超出这个限制将不执行

  int num_sm = device_props.multiProcessorCount;                   // 获取SM数量
  int max_blocks_per_sm = device_props.maxBlocksPerMultiProcessor; // 每个SM上的最大线程块数
  int max_blocks = num_sm * max_blocks_per_sm;                     // 可以使用的最大线程块数量
  printf("Device %d properties:\n", device_id);
  printf("  Number of SMs: %d\n", num_sm);
  printf("  Max blocks per SM: %d\n", max_blocks_per_sm);
  printf("  Max blocks: %d\n", max_blocks);
  // Device 0 properties:
  //   Number of SMs: 30
  //   Max blocks per SM: 16
  //   Max blocks: 480
  // 看出你的GPU具有30个SM，每个SM上最多可以运行16个线程块，因此可以使用的最大线程块数量为480
  // 但是实际上用的时候我超出这个限制也没报错
}
