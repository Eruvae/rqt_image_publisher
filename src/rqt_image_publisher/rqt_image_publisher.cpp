#include "rqt_image_publisher/rqt_image_publisher.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QFileDialog>
#include <QImage>
#include <QThread>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace rqt_image_publisher
{

RqtImagePublisherWidget::RqtImagePublisherWidget(RqtImagePublisher* plugin, QWidget* parent, Qt::WindowFlags f)
  : QWidget(parent, f), plugin(plugin)
{
}

void RqtImagePublisherWidget::resizeEvent(QResizeEvent *event)
{
  if (plugin->image_qpix.isNull())
      return;

  int w = plugin->ui.imageView->width();
  int h = plugin->ui.imageView->height();
  plugin->ui.imageView->setPixmap(plugin->image_qpix.scaled(w,h,Qt::KeepAspectRatio));
}

RqtImagePublisher::RqtImagePublisher() :
  rqt_gui_cpp::Plugin(),
  widget(Q_NULLPTR),
  folder_model(Q_NULLPTR)
{
  setObjectName("RqtImagePublisher");
}

void RqtImagePublisher::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget = new RqtImagePublisherWidget(this);
  // extend the widget with all attributes and children from UI file
  ui.setupUi(widget);
  ui.settingsWidget->hide();
  // add widget to the user interface
  context.addWidget(widget);

  imt = new image_transport::ImageTransport(getNodeHandle());

  image_pub = imt->advertise("/camera/color/image_raw", 1);

  connect(ui.selectFolderButton, SIGNAL(clicked()), this, SLOT(on_selectFolderButton_clicked()));
  connect(ui.fileTreeView, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(on_fileTreeView_doubleClicked(QModelIndex)));
  connect(ui.previousImageButton, SIGNAL(clicked()), this, SLOT(on_previousImageButton_clicked()));
  connect(ui.nextImageButton, SIGNAL(clicked()), this, SLOT(on_nextImageButton_clicked()));
  connect(ui.publishButton, SIGNAL(clicked()), this, SLOT(on_publishButton_clicked()));
  connect(ui.openSettingsButton, SIGNAL(clicked()), this, SLOT(on_openSettingsButton_clicked()));
  connect(ui.settingsApplyButton, SIGNAL(clicked()), this, SLOT(on_settingsApplyButton_clicked()));
  connect(ui.settingsCancelButton, SIGNAL(clicked()), this, SLOT(on_settingsCancelButton_clicked()));
  connect(ui.publishContinouslyRadioButton, SIGNAL(toggled(bool)), this, SLOT(on_publishContinouslyRadioButton_toggled(bool)));
  connect(ui.rotateImagesCheckBox, SIGNAL(toggled(bool)), this, SLOT(on_rotateImagesCheckBox_toggled(bool)));
}

void RqtImagePublisher::shutdownPlugin()
{
  // unregister all publishers here
}

void RqtImagePublisher::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void RqtImagePublisher::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

void RqtImagePublisher::on_selectFolderButton_clicked()
{
  QString dir_path = QFileDialog::getExistingDirectory(widget);
  if (dir_path.isEmpty())
    return;

  ui.imageView->clear();
  image_qimg = QImage();
  image_qpix = QPixmap();
  ui.previousImageButton->setEnabled(false);
  ui.nextImageButton->setEnabled(false);
  ui.publishButton->setEnabled(false);
  ui.curImageLabel->setText("0/0");

  QFileSystemModel *model = new QFileSystemModel;
  model->setRootPath(dir_path);
  ui.fileTreeView->setModel(model);
  ui.fileTreeView->setRootIndex(model->index(dir_path));
  ui.fileTreeView->hideColumn(1);
  ui.fileTreeView->hideColumn(2);
  ui.fileTreeView->hideColumn(3);

  if (folder_model)
    delete folder_model;

  folder_model = model;
}

void RqtImagePublisher::on_fileTreeView_doubleClicked(const QModelIndex &index)
{
  loadImage(index);
}

void RqtImagePublisher::on_previousImageButton_clicked()
{
  if (!selected_image.isValid())
    return;

  int num_imgs_in_folder = folder_model->rowCount(selected_image.parent());
  int item_row = selected_image.row();

  auto previous_row = [&](int row) { return row > 0 ? row - 1 : num_imgs_in_folder - 1; };

  for (int cur_row = previous_row(item_row); cur_row != item_row; cur_row = previous_row(cur_row))
  {
    QModelIndex index = folder_model->index(cur_row, selected_image.column(), selected_image.parent());
    if (loadImage(index))
      break;
  }
}

void RqtImagePublisher::on_nextImageButton_clicked()
{
  if (!selected_image.isValid())
    return;

  int num_imgs_in_folder = folder_model->rowCount(selected_image.parent());
  int item_row = selected_image.row();

  auto next_row = [&](int row) { return (row + 1) % num_imgs_in_folder; };

  for (int cur_row = next_row(item_row); cur_row != item_row; cur_row = next_row(cur_row))
  {
    QModelIndex index = folder_model->index(cur_row, selected_image.column(), selected_image.parent());
    if (loadImage(index))
      break;
  }
}

void RqtImagePublisher::on_publishButton_clicked()
{
  image_ros.header.stamp = ros::Time::now();
  image_pub.publish(image_ros);
}

void RqtImagePublisher::on_openSettingsButton_clicked()
{
  ui.settingsWidget->show();
}

void RqtImagePublisher::on_settingsCancelButton_clicked()
{
  ui.settingsWidget->hide();
}

void RqtImagePublisher::on_settingsApplyButton_clicked()
{
  ui.settingsWidget->hide();
}

void RqtImagePublisher::on_publishContinouslyRadioButton_toggled(bool checked)
{
  ui.publishingFrequencySpinBox->setEnabled(checked);
}

void RqtImagePublisher::on_rotateImagesCheckBox_toggled(bool checked)
{
  ui.publishOnceRadioButton->setEnabled(!checked);
  ui.publishLatchedRadioButton->setEnabled(!checked);
  ui.publishContinouslyRadioButton->setEnabled(!checked);
  ui.publishingFrequencySpinBox->setEnabled(!checked && ui.publishContinouslyRadioButton->isChecked());

  ui.rotateBackwardsCheckBox->setEnabled(checked);
  ui.diashowFrequencySpinBox->setEnabled(checked);
}


bool RqtImagePublisher::loadImage(const QModelIndex &index)
{
  QString path = folder_model->filePath(index);
  QImage new_image;
  if (!new_image.load(path))
  {
    ROS_INFO_STREAM("Selected file is not an image");
    return false;
  }

  image_qimg = new_image;

  // store loaded image as pixmap for display
  image_qpix = QPixmap::fromImage(image_qimg);
  int w = ui.imageView->width();
  int h = ui.imageView->height();
  ui.imageView->setPixmap(image_qpix.scaled(w,h,Qt::KeepAspectRatio));

  // convert image to ROS message for publishing
  QImage qt_image_rgb8 = image_qimg.convertToFormat(QImage::Format_RGB888);
  image_ros.height = qt_image_rgb8.height();
  image_ros.width = qt_image_rgb8.width();
  image_ros.encoding = sensor_msgs::image_encodings::RGB8;
  image_ros.step = qt_image_rgb8.bytesPerLine();
  image_ros.data.resize(qt_image_rgb8.byteCount());
  memcpy(image_ros.data.data(), qt_image_rgb8.bits(), qt_image_rgb8.byteCount());
  image_ros.header.stamp = ros::Time::now();

  if (ui.publishOnLoadCheckBox->isChecked())
    image_pub.publish(image_ros);

  selected_image = index;
  int num_imgs_in_folder = folder_model->rowCount(selected_image.parent());
  int item_row = selected_image.row();
  ui.curImageLabel->setText(QString::asprintf("%d/%d", item_row + 1, num_imgs_in_folder));
  ui.fileTreeView->setCurrentIndex(selected_image);

  ui.previousImageButton->setEnabled(true);
  ui.nextImageButton->setEnabled(true);
  ui.publishButton->setEnabled(true);

  return true;
}

} // namespace rqt_image_publisher

PLUGINLIB_EXPORT_CLASS(rqt_image_publisher::RqtImagePublisher, rqt_gui_cpp::Plugin)
//PLUGINLIB_DECLARE_CLASS(rqt_image_publisher, RqtImagePublisher, rqt_image_publisher::RqtImagePublisher, rqt_gui_cpp::Plugin)
