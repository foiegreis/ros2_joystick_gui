#ifndef JOYSTICK_GUI_CPP_JOYSTICK_WIDGET_HPP
#define JOYSTICK_GUI_CPP_JOYSTICK_WIDGET_HPP

#include <QWidget>
#include <QPointF>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>

class JoystickWidget : public QWidget {
  Q_OBJECT

public:
  explicit JoystickWidget(double init_max_linear = 1.0, double init_max_angular = 1.0, bool invert_angular = false, QWidget *parent = nullptr);
  ~JoystickWidget();

  void setMaxLinearVelocity(double value);
  void setMaxAngularVelocity(double value);

  // Invert angular variable made public for access from JoystickNode
  bool invert_angular_;

  void emitValues();  // Moved to public access specifier

signals:
  void joystickMoved(float linear, float angular);
  void maxLinearVelocityChanged(double value);
  void maxAngularVelocityChanged(double value);

private slots:
  void updateMaxLinear(double value);
  void updateMaxAngular(double value);

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;

private:
  // Joystick parameters
  QPointF center_;
  QPointF handle_pos_;
  float outer_radius_;
  float inner_radius_;
  bool is_dragging_;

  // Max velocities
  double max_linear_velocity_;
  double max_angular_velocity_;

  // UI elements
  QLabel *linear_label_;
  QLabel *angular_label_;
  QDoubleSpinBox *max_linear_spinbox_;
  QDoubleSpinBox *max_angular_spinbox_;
};

#endif  // JOYSTICK_GUI_CPP_JOYSTICK_WIDGET_HPP
