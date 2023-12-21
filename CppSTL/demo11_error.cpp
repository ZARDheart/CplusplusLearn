#include <iostream>
#include <exception>
using namespace std;

/*  非常简单的C++异常处理例子，不足以用于学习
C++标准程序库有两种方式：
    标准程序库中有一部分，例如std classes，支持具体的错误处理，他们检测所有可能发生的错误，并于错误发生时抛出异常
    像STL，valarrays等，效率重于安全，因此几乎不检验逻辑错误，并且只检测执行期(runtime)错误
C++ 标准库提供了许多预定义的异常类，称为内置异常，包括以下几种：
    std::exception：所有标准异常类的基类。
    std::logic_error：表示程序逻辑错误。
    std::runtime_error：表示运行时错误。
     异常的处理方式
C++ 提供了以下几种处理异常的方式：
    1 try-catch 语句
    使用 try 代码块来包含可能抛出异常的代码，紧跟 catch 块来处理捕获到的异常。
    当抛出异常后，程序会自动匹配 catch 块中与对应异常相同类型的代码块执行，保证程序正常执行。
        try {
        // 可能会抛出异常的代码块
        } catch (exception1_type exception1_var) {
        // 处理异常类型1
        } catch (exception2_type exception2_var) {
        // 处理异常类型2

    2 throw 语句
    使用 throw 语句抛出一个异常对象，使程序进入异常处理模式。
    exception_object 可以是基本类型或对象，甚至可以是自定义类型的实例。
        throw exception_object;

    3 noexcept 修饰符
    noexcept 修饰符指示函数不抛出异常。使用 noexcept 可以优化程序性能。当程序遇到一个没有 noexcept 修饰符的函数，会假设这个函数可能抛出异常，导致额外的代码执行。
        void function_name() noexcept {

    4 finally 语句块
    finally 语句在 try-catch 语句的末尾执行，无论异常是否抛出。通常用于释放资源等程序收尾工作。
        try{
        // 可能抛出异常的代码块
        } catch(...) {
        // 处理异常
        } finally {
        // 无论有无异常都执行
        }
*/

// 自定义一个异常类
class DivideByZeroException : public exception
{
public:
    // 重写 what() 函数，返回异常信息
    const char *what() const noexcept override
    {
        return "Attempt to divide by zero";
    }
};

// 除法函数，在该函数中可能会抛出一个 DivideByZeroException 异常
double divide(double a, double b)
{
    if (b == 0)
    {
        throw DivideByZeroException();
    }
    return a / b;
}

int main()
{
    double a = 42, b = 0, result;
    try
    {
        result = divide(a, b);
        cout << "Result is: " << result << endl;
    }
    catch (const DivideByZeroException &e)
    {
        cerr << "Exception caught: " << e.what() << endl;
    }
    cout << "Program continues to run" << endl;
    return 0;
}