#ifndef RQT_IMAGE_PUBLISHER_H
#define RQT_IMAGE_PUBLISHER_H

#include <QWidget>
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include "ui_rqt_image_publisher.h"
#include <image_transport/image_transport.h>
#include <QFileSystemModel>

namespace rqt_image_publisher
{

struct PluginSettings
{
  QString imageTopic;
  QString frameId;
  bool publishOnLoad;
  bool publishLatched;
  bool publishContinously;
  double publishingFrequency;
  bool rotateImages;
  bool rotateBackwards;
  double rotationFrequency;
  bool scaleWidth;
  int width;
  bool scaleHeight;
  int height;
  bool keepRatio;
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
  void on_settingsCancelButton_clicked();
  void on_settingsApplyButton_clicked();
  void on_publishContinouslyCheckBox_toggled(bool checked);
  void on_rotateImagesCheckBox_toggled(bool checked);

signals:

private:
  bool loadImage(const QModelIndex &index);
  bool generateRosImage();
  void pluginSettingsToUi();
  void uiToPluginSettings();
  void applySettings();

  Ui::RqtImagePublisher ui;
  RqtImagePublisherWidget* widget;
  image_transport::ImageTransport *imt;
  image_transport::Publisher image_pub;

  QFileSystemModel *folder_model;

  QModelIndex selected_image;
  QImage image_qimg;
  QPixmap image_qpix;
  sensor_msgs::Image image_ros;
  PluginSettings settings;
};

}

#endif // RQT_IMAGE_PUBLISHER_H
