
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QListWidget>
#include <QMessageBox>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void startTurtlesim();  // Función para iniciar turtlesim
    void stopTurtlesim();
    //void MainWindow::onConnectButtonClicked();


private:
    Ui::MainWindow *ui;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    bool turtlesim_started_;

    void initROS();
    void setupConnections();
};

#endif // MAINWINDOW_H

