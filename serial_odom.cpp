#include <boost/asio.hpp>
#include <iostream>
#include <random>
int main() {
	boost::asio::io_service io_service;
	boost::asio::serial_port port(io_service, "COM3"); // 串口号为COM3
	port.set_option(boost::asio::serial_port_base::baud_rate(115200)); // 波特率为9600

	std::uniform_real_distribution<float> dis(-0.5, 0.5);
	std::random_device rd;
	while (true) {
		float vx =dis(rd) , wz = dis(rd);
		float ze = 0;
		float check_sum = vx + wz;
		port.write_some(boost::asio::buffer(&ze, sizeof(float))); // 发送第一个浮点数
		port.write_some(boost::asio::buffer(&vx, sizeof(float))); // 发送第二个浮点数
		port.write_some(boost::asio::buffer(&wz, sizeof(float))); // 发送第二个浮点数
		port.write_some(boost::asio::buffer(&check_sum, sizeof(float))); // 发送第二个浮点数

		float real_vx, real_wz, real_check_sum,real_zero;
		boost::asio::read(port, boost::asio::buffer(&real_zero, sizeof(real_zero)));
		boost::asio::read(port, boost::asio::buffer(&real_vx, sizeof(real_vx)));
		boost::asio::read(port, boost::asio::buffer(&real_wz, sizeof(real_wz)));
		boost::asio::read(port, boost::asio::buffer(&real_check_sum, sizeof(real_check_sum)));

		if (fabs(real_check_sum - real_vx - real_wz - real_zero) < 0.01) {
			std::cout << "check pass: " << real_vx<<" vs  "<<vx << std::endl;
		}

		
	}
	

	return 0;
}
