#include "joystick_gui_cpp/joystick_widget.hpp"
#include <QPainter>
#include <QMouseEvent>
#include <cmath>

JoystickWidget::JoystickWidget(double init_max_linear, double init_max_angular, bool invert_angular, QWidget *parent)
    : QWidget(parent),
      outer_radius_(100.0f),
      inner_radius_(20.0f),
      is_dragging_(false),
      max_linear_velocity_(init_max_linear),
      max_angular_velocity_(init_max_angular),
      invert_angular_(invert_angular) {
  setFixedSize(300, 400);  // Increased height for additional widgets

  center_ = QPointF(width() / 2, 150);  // Adjusted center position
  handle_pos_ = center_;

  // Create labels
  linear_label_ = new QLabel("Linear Velocity: 0.0 m/s", this);
  angular_label_ = new QLabel("Angular Velocity: 0.0 rad/s", this);

  // Create spin boxes for max velocities
  max_linear_spinbox_ = new QDoubleSpinBox(this);
  max_linear_spinbox_->setRange(0.1, 10.0);
  max_linear_spinbox_->setValue(max_linear_velocity_);
  max_linear_spinbox_->setSingleStep(0.1);
  max_linear_spinbox_->setPrefix("Max Linear: ");
  max_linear_spinbox_->setSuffix(" m/s");

  max_angular_spinbox_ = new QDoubleSpinBox(this);
  max_angular_spinbox_->setRange(0.1, 10.0);
  max_angular_spinbox_->setValue(max_angular_velocity_);
  max_angular_spinbox_->setSingleStep(0.1);
  max_angular_spinbox_->setPrefix("Max Angular: ");
  max_angular_spinbox_->setSuffix(" rad/s");

  // Connect spin boxes to slots and emit signals
  connect(max_linear_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JoystickWidget::updateMaxLinear);
  connect(max_angular_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &JoystickWidget::updateMaxAngular);

  // Layout widgets
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->setAlignment(Qt::AlignTop);

  // Add the joystick area (we'll leave it as part of the widget's paint event)
  QWidget *joystick_area = new QWidget(this);
  joystick_area->setFixedSize(300, 300);

  main_layout->addWidget(joystick_area);

  // Add labels and spin boxes
  main_layout->addWidget(linear_label_);
  main_layout->addWidget(angular_label_);
  main_layout->addWidget(max_linear_spinbox_);
  main_layout->addWidget(max_angular_spinbox_);

  setLayout(main_layout);
}

JoystickWidget::~JoystickWidget() {}

void JoystickWidget::paintEvent(QPaintEvent *event) {
  Q_UNUSED(event);
  QPainter painter(this);

  // Draw outer circle
  painter.setPen(QPen(Qt::black, 2));
  painter.drawEllipse(center_, outer_radius_, outer_radius_);

  // Draw inner circle (joystick handle)
  painter.setBrush(QBrush(QColor(200, 0, 0)));
  painter.drawEllipse(handle_pos_, inner_radius_, inner_radius_);
}

void JoystickWidget::mousePressEvent(QMouseEvent *event) {
  if (QLineF(event->pos(), handle_pos_).length() < inner_radius_) {
    is_dragging_ = true;
  }
}

void JoystickWidget::mouseMoveEvent(QMouseEvent *event) {
  if (is_dragging_) {
    QPointF offset = event->pos() - center_;
    float distance = std::sqrt(offset.x() * offset.x() + offset.y() * offset.y());
    if (distance > outer_radius_) {
      // Constrain within the outer circle
      float angle = std::atan2(offset.y(), offset.x());
      offset.setX(outer_radius_ * std::cos(angle));
      offset.setY(outer_radius_ * std::sin(angle));
    }
    handle_pos_ = center_ + offset;
    update();
    emitValues();
  }
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent *event) {
  Q_UNUSED(event);
  is_dragging_ = false;
  handle_pos_ = center_;  // Reset to center
  update();
  emitValues();
}

void JoystickWidget::emitValues() {
  // Calculate normalized values from -1 to 1
  float dx = (handle_pos_.x() - center_.x()) / outer_radius_;
  float dy = (center_.y() - handle_pos_.y()) / outer_radius_;

  // Invert angular direction if required
  if (invert_angular_) {
    dx = -dx;
  }

  // Scale with max velocities
  float linear_velocity = dy * max_linear_velocity_;
  float angular_velocity = dx * max_angular_velocity_;

  // Update labels
  linear_label_->setText(QString("Linear Velocity: %1 m/s").arg(linear_velocity, 0, 'f', 2));
  angular_label_->setText(QString("Angular Velocity: %1 rad/s").arg(angular_velocity, 0, 'f', 2));

  // Emit the scaled values
  emit joystickMoved(linear_velocity, angular_velocity);
}

void JoystickWidget::updateMaxLinear(double value) {
  max_linear_velocity_ = value;
  emit maxLinearVelocityChanged(value);
}

void JoystickWidget::updateMaxAngular(double value) {
  max_angular_velocity_ = value;
  emit maxAngularVelocityChanged(value);
}

void JoystickWidget::setMaxLinearVelocity(double value) {
  max_linear_velocity_ = value;
  max_linear_spinbox_->setValue(value);
}

void JoystickWidget::setMaxAngularVelocity(double value) {
  max_angular_velocity_ = value;
  max_angular_spinbox_->setValue(value);
}
