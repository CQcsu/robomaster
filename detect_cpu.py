import multiprocessing
import time

# 检测 cpu能打开的最多进程


if __name__ == '__main__':
    #
    num_threads = multiprocessing.cpu_count()
    print("cpu的核数：", num_threads)
    print(num_threads, "核", num_threads*2, "线程")

