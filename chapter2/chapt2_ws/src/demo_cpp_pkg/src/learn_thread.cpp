#include <iostream> // 用于标准输入输出（std::cout）
#include <thread>   // 用于 std::thread、std::this_thread::get_id、std::this_thread::sleep_for
#include <chrono>   // 用于时间间隔 std::chrono::milliseconds
#include <functional> // 用于 std::function 和 std::bind
#include <cpp-httplib/httplib.h> // 第三方单头文件 HTTP 客户端库，用于发起 HTTP GET 请求

// Download 类封装了一个简单的下载功能：从指定 host + path 获取资源，并在完成后调用回调函数。

// - download 方法在调用时同步执行（可从任意线程调用），完成后通过回调返回数据。
// - start_download 方法通过 std::thread 启动一个新线程执行 download，然后 detach 线程使其在后台运行。
// - 回调类型 std::function<void(const std::string&, const std::string&)>：
//   第一个参数为完整 URL（host + path），第二个参数为下载到的内容 body。
class Download
{
public:
    // download: 执行实际的 HTTP GET 请求并在成功时回调
    // 参数:
    // - host: 主机地址（例如 "http://localhost:8080"）
    // - path: 路径部分（例如 "/novel1.txt"）
    // - callback: 下载完成后的回调，签名为 (const std::string &path, const std::string &result)
    void download(const std::string &host, const std::string &path, 
                  const std::function<void(const std::string &, const std::string &)> &callback)
    {
        // 输出当前线程 ID 以便观察是哪个线程在执行下载
        std::cout << "线程ID: " << std::this_thread::get_id() << " 开始下载文件 from " << host + path << std::endl;

        httplib::Client client(host);

        // 发起 GET 请求
        auto response = client.Get(path);

        // 检查响应是否有效且状态码为 200（OK）
        if (response && response->status == 200)
        {
            // 成功则通过回调通知调用者，传递完整 URL 和响应体内容
            callback(host + path, response->body);
        }
    }

    // start_download: 在新的分离线程中启动下载任务并立即返回
    // 参数与 download 相同；此方法将下载工作放到后台线程执行，不会阻塞调用者线程。
    void start_download(const std::string &host, const std::string &path, 
                        const std::function<void(const std::string &, const std::string &)> &callback)
    {
        // 使用 std::bind 绑定成员函数 download 到当前对象（this）。
        // bind 生成一个可调用对象 download_fun，接受 (host, path, callback) 参数。
        auto download_fun = std::bind(&Download::download, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        // 使用 std::thread 启动新线程执行 download_fun。
        // 传递 host, path, callback 作为线程函数的参数。
        std::thread download_thread(download_fun, host, path, callback);

        // detach() 将线程分离为后台线程，使其在完成后自动释放资源。
        // 注意：分离线程的生命周期不受创建它的线程控制，必须确保主程序不会过早退出。
        download_thread.detach();
    }
};

int main()
{
    // 创建 Download 对象用于发起下载
    Download download;

    // 定义下载完成时的回调函数（lambda）
    // 参数:
    // - path: 完整 URL（host + path）
    // - result: 下载到的文件内容（字符串）
    // 该回调在下载线程中被调用，所以如果回调中有额外线程不安全操作需注意同步。
    auto download_finish_callback = [](const std::string &path, const std::string &result) -> void {
        std::cout << "文件下载完成 from " << path << ", 文件大小: " << result.size() << " 字节" << std::endl;
    };

    // 启动三个并发下载任务（均为分离线程）
    download.start_download("http://localhost:8000", "/novel1.txt", download_finish_callback);
    download.start_download("http://localhost:8000", "/novel2.txt", download_finish_callback);
    download.start_download("http://localhost:8000", "/novel3.txt", download_finish_callback);

    // 主线程等待一段时间，保证后台分离线程有足够时间完成下载并调用回调。
    // 在真实程序中应使用更稳健的同步机制（例如 future/promise、condition_variable 或 joinable 的线程）来管理线程生命周期。
    std::this_thread::sleep_for(std::chrono::milliseconds(1000*10)); // 等待 10 秒

    return 0;
}
