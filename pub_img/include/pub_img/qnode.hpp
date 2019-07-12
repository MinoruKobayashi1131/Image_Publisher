/**
 * @file /include/pub_img/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef pub_img_QNODE_HPP_
#define pub_img_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <time.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pub_img {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
  QString movie_name;
  cv::Mat video_buf;

Q_SIGNALS:
    void rosShutdown();
    void videoUpdated();

private:
	int init_argc;
	char** init_argv;
  image_transport::Publisher image_pub;
};

}  // namespace pub_img

#endif /* pub_img_QNODE_HPP_ */
