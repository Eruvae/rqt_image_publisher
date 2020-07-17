#ifndef RQT_IMAGE_PUBLISHER_H
#define RQT_IMAGE_PUBLISHER_H

#include <QWidget>
#include <QFileSystemModel>
#include <QTimer>
#include <QImage>
#include <QPixmap>
#include <QModelIndex>
#include <QString>
#include <QStringList>

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <unordered_set>

#include "ui_rqt_image_publisher.h"

namespace rqt_image_publisher
{

enum ImageType
{
  IT_INVALID = -1,
  IT_AUTO = 0,
  IT_COLOR = 1,
  IT_COLOR_ALPHA = 2,
  IT_MONO = 3,
  IT_DEPTH = 4
};

enum Encoding
{
  ENC_INVALID = -1,
  ENC_AUTO = 0,
  ENC_RGB8 = 1,
  ENC_RGB16 = 2,
  ENC_BGR8 = 3,
  ENC_BGR16 = 4,
  ENC_RGBA8 = 5,
  ENC_RGBA16 = 6,
  ENC_BGRA8 = 7,
  ENC_BGRA16 = 8,
  ENC_MONO8 = 9,
  ENC_MONO16 = 10,
  ENC_16UC1 = 11,
  ENC_32FC1 = 12
};

static inline std::string encodingToString(Encoding enc)
{
  using namespace sensor_msgs::image_encodings;
  switch (enc)
  {
  case ENC_RGB8: return RGB8;
  case ENC_RGB16: return RGB16;
  case ENC_BGR8: return BGR8;
  case ENC_BGR16: return BGR16;
  case ENC_RGBA8: return RGBA8;
  case ENC_RGBA16: return RGBA16;
  case ENC_BGRA8: return BGRA8;
  case ENC_BGRA16: return BGRA16;
  case ENC_MONO8: return MONO8;
  case ENC_MONO16: return MONO16;
  case ENC_16UC1: return TYPE_16UC1;
  case ENC_32FC1: return TYPE_32FC1;
  default: return "";
  }
}

static inline Encoding stringToEncoding(const std::string &enc)
{
  using namespace sensor_msgs::image_encodings;
  if (enc == RGB8) return ENC_RGB8;
  if (enc == RGB16) return ENC_RGB16;
  if (enc == BGR8) return ENC_BGR8;
  if (enc == BGR16) return ENC_BGR16;
  if (enc == RGBA8) return ENC_RGBA8;
  if (enc == RGBA16) return ENC_RGBA16;
  if (enc == BGRA8) return ENC_BGRA8;
  if (enc == BGRA16) return ENC_BGRA16;
  if (enc == MONO8) return ENC_MONO8;
  if (enc == MONO16) return ENC_MONO16;
  if (enc == TYPE_16UC1) return ENC_16UC1;
  if (enc == TYPE_32FC1) return ENC_32FC1;
  else return ENC_INVALID;
}

static inline Encoding indexToEncoding(ImageType type, int index)
{
  int offset = 0;
  if (index > 0)
  {
    switch (type)
    {
    case IT_COLOR: offset = 0; break;
    case IT_COLOR_ALPHA: offset = 4; break;
    case IT_MONO: offset = 8; break;
    case IT_DEPTH: offset = 10; break;
    }
  }
  return (Encoding)(index + offset);
}

static inline int encodingToIndex(ImageType type, Encoding enc)
{
  int offset = 0;
  if (enc > 0)
  {
    switch (type)
    {
    case IT_COLOR: offset = 0; break;
    case IT_COLOR_ALPHA: offset = 4; break;
    case IT_MONO: offset = 8; break;
    case IT_DEPTH: offset = 10; break;
    }
  }
  return (int)enc - offset;
}

struct PluginSettings
{
  QString imageTopic;
  QString frameId;
  bool generateCameraInfo;
  QString cameraInfoTopic;
  ImageType imageType;
  Encoding colorEnc;
  Encoding colorAlphaEnc;
  Encoding monoEnc;
  Encoding depthEnc;
  bool publishOnLoad;
  bool publishLatched;
  bool publishContinuously;
  double publishingFrequency;
  bool rotateImages;
  bool rotateBackwards;
  bool startSlideshowOnLoad;
  double rotationFrequency;
  bool scaleWidth;
  int width;
  bool scaleHeight;
  int height;
  bool keepRatio;
  bool scaleDepth;
  bool dynamicDepthRange;
  double depthMinRange;
  double depthMaxRange;
  QStringList filters;
  QString lastFolder;

  bool synchronizePublishing;
};

class RqtImagePublisher;

class RqtImagePublisherWidget : public QWidget
{
public:
  explicit RqtImagePublisherWidget(RqtImagePublisher* plugin, QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());

protected:
  virtual void resizeEvent(QResizeEvent *event);

private:
  RqtImagePublisher *plugin;
};

class RqtImagePublisher : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

  friend class RqtImagePublisherWidget;

public:
  RqtImagePublisher();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
  virtual bool hasConfiguration() const;
  virtual void triggerConfiguration();

private slots:
  void on_selectFolderButton_clicked();
  void on_fileTreeView_doubleClicked(const QModelIndex &index);
  void on_nextImageButton_clicked();
  void on_previousImageButton_clicked();
  void on_publishButton_clicked();
  void on_openSettingsButton_clicked();
  void on_filterListButton_clicked();
  void on_settingsCancelButton_clicked();
  void on_settingsApplyButton_clicked();
  void on_settingsOkButton_clicked();
  void on_filterListCancelButton_clicked();
  void on_filterListApplyButton_clicked();
  void on_cameraInfoCheckBox_toggled(bool checked);
  void on_scaleWidthCheckBox_toggled(bool checked);
  void on_scaleHeightCheckBox_toggled(bool checked);
  void on_scaleDepthCheckBox_toggled(bool checked);
  void on_dynamicRangeCheckbox_toggled(bool checked);
  void on_publishContinuouslyCheckBox_toggled(bool checked);
  void on_rotateImagesCheckBox_toggled(bool checked);
  void on_minRangeSpinBox_valueChanged(double value);
  void on_maxRangeSpinBox_valueChanged(double value);
  void on_publishingTimer_timeout();
  void on_synchronizedTimeoutSignal_received(const ros::Time &time);
  void on_synchronizedSettings_changed(const PluginSettings &shared_settings);

signals:
  void synchronizedTimeoutSignal(const ros::Time &time);
  void synchronizedSettingsChanged(const PluginSettings &shared_settings);

private:
  // Button text constants
  static const QString PUBLISH;
  static const QString START_PUBLISHING;
  static const QString STOP_PUBLISHING;
  static const QString START_SLIDESHOW;
  static const QString STOP_SLIDESHOW;

  bool loadImage(const QModelIndex &index);
  cv::Mat convertToDepth(const cv::Mat &mat, int depth);
  cv::Mat convertDepthForDisplay(const cv::Mat &mat);
  void removeDepthOutliers(cv::Mat &mat);
  cv::Mat convertToEncoding(const cv::Mat &mat, Encoding enc, Encoding sourceEnc = ENC_AUTO);
  void resizeCvBridgeImage();
  ImageType detectType(const cv::Mat &mat);
  Encoding detectEncoding(const cv::Mat &mat, ImageType type);
  bool generateCvBridgeImage();
  void generateCameraInfo();
  void generatePixmap();
  void publishImage(const ros::Time &time = ros::Time::now());
  static void synchronizedPublishImage();
  //bool generateRosImage();
  void setSelectedImage(QModelIndex index);
  void clearSelectedImage();
  void startSlideshow();
  void stopSlideshow();
  void startPublishing();
  void stopPublishing();
  static void synchronizedStartSlideshow();
  static void synchronizedStopSlideshow();
  static void synchronizedStartPublishing();
  static void synchronizedStopPublishing();
  void pluginSettingsToUi();
  void uiToPluginSettings();
  void rescaleImageLabel();
  void applySettings();
  void applySyncedSettings();

  Ui::RqtImagePublisher ui;
  RqtImagePublisherWidget* widget;
  image_transport::ImageTransport *imt;
  image_transport::Publisher image_pub;
  ros::Publisher camera_info_pub;
  QTimer publishingTimer;

  QFileSystemModel *folder_model;

  bool image_loaded;
  QModelIndex selected_image;
  cv::Mat image_cv;
  Encoding image_encoding;
  cv_bridge::CvImage image_cvb;
  QImage image_qimg;
  QPixmap image_qpix;
  sensor_msgs::Image image_ros;
  sensor_msgs::CameraInfo camera_info;
  PluginSettings settings;

  static bool synchronizeSettings;
  static std::unordered_set<RqtImagePublisher*> plugin_instances;
  static QTimer synchronizedTimer;
  static void notifyInstances();
  void notifyChangedSettings();
  void notifyPublishButtonClicked();
};

}

#endif // RQT_IMAGE_PUBLISHER_H
