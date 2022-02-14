#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include "QtSerialPort/QSerialPortInfo"
#include <QTimer>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QHostAddress>
#include <QMessageBox>

typedef union{
    uint8_t     u8[24];
    int8_t      i8[4];
    uint16_t    u16[24];
    int16_t     i16[2];
    uint32_t    u32;
    int32_t     i32;
    float       f;
}_work;


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    _work w, distancia_us;

private slots:

    void OnQUdpSocket1Rx();
    void OnQUdpSocket1Tx();

    void OnQSerialPort1Rx();
    void on_EnviarTx_clicked();

    void OnQTimer1();
    void on_pushButton_clicked();

    void DecodeHeader(uint8_t *buf,uint8_t count);

    void on_pushButton_6_clicked();

    void on_comboBox_2_activated(const QString &arg1);

    void on_pushButton_3_clicked();

    //void on_horizontalSlider_valueChanged(int value);

    //void on_horizontalSlider_sliderMoved(int position);

    void on_verticalSlider_2_valueChanged(int value);

    void on_verticalSlider_valueChanged(int value);

    //void on_horizontalSlider_actionTriggered(int action);

    //void on_verticalSlider_4_valueChanged(int value);

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

private:
    Ui::MainWindow *ui;

    QTimer *QTimer1;
    QSerialPort *QSerialPort1;

    uint8_t rx[256], header, timeout, nbytes, cks, index, time1, valuechanged_1, valuechanged_2;

    QUdpSocket *QUdpSocket1;
    QHostAddress hostAddress;     //Direccion de quien envio
    quint16 hostPort;             // Puerto de quien envio

    uint8_t buff_UDP[256];
    uint16_t indW_UDP=0,indR_UDP=0;

    void DecodeCmd(uint8_t *RX);
    void SendCmd(uint8_t *buf, uint8_t length);
};
#endif // MAINWINDOW_H
