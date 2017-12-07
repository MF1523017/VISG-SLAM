
#include "camera.h"
#include "test.h"

int main(int argc, char **argv) {
//	test_Camera()
	if (argc == 1)
		test_VisgSlam();
	else if (argc == 2) {
		if ('r' == argv[1][0])
			test_VisgSlam(argv);
		else
			test_VisgSlamOffline(argv);
	}
	std::cout << "usage: " << argv[0] << " (for running stereo slam) or " << argv[0] << " data_dir (for recording data)" << std::endl;
	system("pause");
    return 0;
}
