#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include "QtSerialPort/QSerialPortInfo"
#include <QTimer>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QHostAddress>
#include <QMessageBox>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

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

    void on_verticalSlider_2_valueChanged(int value);

    void on_verticalSlider_valueChanged(int value);

    void on_pushButton_5_clicked();

    void on_lineEdit_2_editingFinished();

    void on_lineEdit_3_editingFinished();


private:
#define FIRSTALIVE flag1.bit.b0

    typedef union{
        uint8_t     u8[4];
        int8_t      i8[4];
        uint16_t    u16[2];
        int16_t     i16[2];
        uint32_t    u32;
        int32_t     i32;
        float       f;
    }_work;

    typedef union{
        struct
        {
            uint8_t b0: 1;
            uint8_t b1: 1;
            uint8_t b2: 1;
            uint8_t b3: 1;
            uint8_t b4: 1;
            uint8_t b5: 1;
            uint8_t b6: 1;
            uint8_t b7: 1;
        }bit;
        uint8_t byte;
    }_sFlag;

    Ui::MainWindow *ui;

    QTimer *QTimer1;
    QSerialPort *QSerialPort1;

    uint8_t rx[256], header, timeout, nbytes, cks, index, Dato;
    uint16_t Counter = 0, time1;

    QUdpSocket *QUdpSocket1;
    QHostAddress hostAddress;     //Direccion de quien envio
    quint16 hostPort;             // Puerto de quien envio

    _work w;
    _sFlag flag1;

    void DecodeCmd(uint8_t *RX);
    void SendCmd(uint8_t *buf, uint8_t length);
};
#endif // MAINWINDOW_H
