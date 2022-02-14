#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtSerialPort/QSerialPort>

MainWindow::MainWindow(QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QTimer1 = new QTimer(this);
    QSerialPort1 = new QSerialPort(this);

    connect(QSerialPort1,&QSerialPort::readyRead,this,&MainWindow::OnQSerialPort1Rx);
    connect(QTimer1, &QTimer::timeout, this, &MainWindow::OnQTimer1);

    QUdpSocket1 = new QUdpSocket(this);
    connect(QUdpSocket1,&QUdpSocket::readyRead,this,&MainWindow::OnQUdpSocket1Rx);

    header = 0;
    ui->comboBox->addItem("Alive", 0xF0);
    ui->comboBox->addItem("Firmware", 0xF1);
    ui->comboBox->addItem("Valor de sensores", 0xA0);
    ui->comboBox->addItem("Test de motores", 0xA1);
    ui->comboBox->addItem("Angulo", 0xA2);
    ui->comboBox->addItem("Distancia", 0xA3);
    ui->comboBox->addItem("Velocidad", 0xA4);

    QTimer1->start(10);
    time1=0;

    ui->lineEdit->setText("30011");
}

MainWindow::~MainWindow()
{
    if(QUdpSocket1->isOpen())
           QUdpSocket1->abort();
    delete QUdpSocket1;

    delete QTimer1;

    delete QSerialPort1;

    delete ui;
}

void MainWindow::OnQUdpSocket1Tx(){

}


void MainWindow::OnQUdpSocket1Rx(){
    int count;
    uint8_t *buf;


    while(QUdpSocket1->hasPendingDatagrams()){
        count=QUdpSocket1->pendingDatagramSize();
        if(count <= 0){
            return;
        }
        buf=new uint8_t[count];

        QUdpSocket1->readDatagram((char *)buf,count,&hostAddress,&hostPort);

        ui->label_12->setText(hostAddress.toString());
        ui->label_15->setText((const QString) hostPort);

        DecodeHeader(buf, count);

        delete [] buf;
    }
}

/*!
 * \brief MainWindow::OnQTimer1
 * Función timer para control de flujo del programa
 */

void MainWindow::OnQTimer1(){
//    uint8_t buf[24];

    time1 ++;

    if(header){
        timeout--;
        if(!timeout){
            header = 0;
        }
    }

//    if(time1 == 19){
//        buf[0] = 0xA4;
//        SendCmd(buf,1);
//        time1 = 0;
//    }

//    if(time1 == 18){
//        buf[0] = 0xA3;
//        SendCmd(buf,1);
//    }
//    if(time1 == 50){
//        time1 = 0;
//        buf[0] = 0XA0;
//        SendCmd(buf,1);
//    }
}

void MainWindow::DecodeHeader(uint8_t *buf, uint8_t count){

    for(int i=0; i<count; i++){
        switch (header) {
            case 0:
                if(buf[i] == 'U'){
                    header = 1;
                    timeout = 5;
                }
            break;
            case 1:
                if(buf[i] == 'N'){
                    header = 2;
                }
                else{
                    header = 0;
                   i--;
                }
             break;
             case 2:
                 if(buf[i] == 'E'){
                     header = 3;
                 }
                 else{
                     header = 0;
                    i--;
                 }
             break;
             case 3:
                 if(buf[i] == 'R'){
                     header = 4;
                 }
                 else{
                     header = 0;
                    i--;
                 }
             break;
             case 4:                            //LENGTH
                 nbytes = buf[i];
                 header = 5;
             break;
             case 5:                            //TOKEN
                 if(buf[i] == ':'){
                     header = 6;
                     index = 0;
                     cks = 'U' ^ 'N' ^ 'E' ^ 'R' ^ nbytes ^ ':';
                 }
                 else{
                     header = 0;
                    i--;
                 }
             break;
             case 6:                            //ID
                 nbytes--;
                 if(nbytes > 0){
                    rx[index++] = buf[i];       //Se pasa el ID
                    cks ^= buf[i];
                  }
                 else{
                     header = 0;
                     if(cks == buf[i]){
                         DecodeCmd(rx);
                         //ui->plainTextEdit->appendPlainText("DECODE CMD");
                     }
                     else
                         ui->plainTextEdit->appendPlainText("ERROR CHECKSUM");
                 }
             break;
             default:
                 header = 0;
        }
    }
}

/*!
 * \brief MainWindow::OnQSerialPort1Rx
 * Funcion para recibir datos por el puerto serie
 */

void MainWindow::OnQSerialPort1Rx(){
    int count;
    uint8_t *buf;
    QString strHex;

    count = QSerialPort1->bytesAvailable(); //Verifico si tengo datos
    if(count <= 0){
        return;
    }
    buf = new uint8_t(count);
    QSerialPort1->read((char *)buf, count);

    DecodeHeader( buf, count);

    delete buf;
}

/*!
 * \brief MainWindow::SendCmd
 * \param buf -> comando+payload
 * \param length -> cantidad de bytes a transmitir
 * Función para enviar header+comando+payload
 */

void MainWindow::SendCmd(uint8_t *buf, uint8_t length){
    uint8_t tx[24];
    uint8_t cks, i;
    QString strHex;

    if(!QSerialPort1->isOpen())
        return;
    tx[0] = 'U';
    tx[1] = 'N';
    tx[2] = 'E';
    tx[3] = 'R';
    tx[4] = length + 1;
    tx[5] = ':';

    memcpy(&tx[6], buf, length);

    cks = 0;
    i = 0;
    for (i=0; i<(length+6); i++) {
        cks ^= tx[i];
    }

    tx[i] = cks;
    QSerialPort1->write((char *)tx, length+7);

}

/*!
 * \brief MainWindow::on_EnviarTx_clicked
 * Función donde se arma el paquete de transmision del comando+payload
 */

void MainWindow::on_EnviarTx_clicked()
{
    uint8_t cmd, buf[24];
    if(ui->comboBox->currentText() == ""){
        return;
    }
    cmd = ui->comboBox->currentData().toInt();
    ui->plainTextEdit->appendPlainText("0x" + (QString("%1").arg(cmd, 2, 16, QChar('0'))).toUpper());

    switch (cmd) {
        case 0xF0:                  //ALIVE
        ui->plainTextEdit->appendPlainText("ALIVE???");
            buf[0] = cmd;
            SendCmd(buf,1);
        break;
        case 0xA0:                  //VALOR DE SENSORES
            ui->plainTextEdit->appendPlainText("SENSORES???");
            buf[0] = cmd;
            SendCmd(buf,1);
        break;
        case 0xA1:
            buf[0] = cmd;
            w.i32 = ui->verticalSlider->value();            //Motor izquierdo
            buf[1]= w.i8[0];
            buf[2]= w.i8[1];
            buf[3]= w.i8[2];
            buf[4]= w.i8[3];
            w.i32 = ui->verticalSlider_2->value();          //Motor derecho
            buf[5]= w.i8[0];
            buf[6]= w.i8[1];
            buf[7]= w.i8[2];
            buf[8]= w.i8[3];
            SendCmd(buf,9);
        break;
//        case 0xA2:                  //Envio ángulo
//            buf[0] = cmd;
//            w.i8[0] = ui->verticalSlider_4->value();
//            buf[1] = w.i8[0];
//            SendCmd(buf,2);
//        break;
        case 0xA3:                  //Pido distancia_us
            ui->plainTextEdit->appendPlainText("DISTANCIA???");
            buf[0] = cmd;
            SendCmd(buf,1);
        break;
        case 0xA4:                  //Pido velocidad 500ms
            ui->plainTextEdit->appendPlainText("VELOCIDAD???");
            buf[0] = cmd;
            SendCmd(buf,1);
        break;case 0xA5:
            buf[0] = cmd;
            SendCmd(buf,1);
        break;
        case 0xF1:                  //FIRMWARE
            ui->plainTextEdit->appendPlainText("FIRMWARE???");
            buf[0] = cmd;
            SendCmd(buf,1);
        break;

    }
}

/*!
 * \brief MainWindow::DecodeCmd
 * \param RX -> buffer de recepcion MBED->PC
 * Función para decodificar el comando+payload
 */

void MainWindow::DecodeCmd(uint8_t *RX){
//    int i = 1;
    QString aux;
    switch(RX[0]){
        case 0xA0:
            w.u8[0] = RX[1];
            w.u8[1] = RX[2];
            w.u8[2] = RX[3];
            w.u8[3] = RX[4];
            w.u8[4] = RX[5];
            w.u8[5] = RX[6];
            w.u8[6] = RX[7];
            w.u8[7] = RX[8];
            w.u8[8] = RX[9];
            w.u8[9] = RX[10];
            w.u8[10] = RX[11];
            w.u8[11] = RX[12];
            w.u8[12] = RX[13];
            w.u8[13] = RX[14];
            w.u8[14] = RX[15];
            w.u8[15] = RX[16];
            ui->label_IR1->clear();
//            ui->label_IR1->setText(QString::number(w.u16[0]));
            ui->progressBar_IR1->setValue((w.u16[0]*100)/4096);
            ui->label_IR2->clear();
//            ui->label_IR2->setText(QString::number(w.u16[1]));
            ui->progressBar_IR2->setValue((w.u16[1]*100)/4096);
            ui->label_IR3->clear();
//            ui->label_IR3->setText(QString::number(w.u16[2]));
            ui->progressBar_IR3->setValue((w.u16[2]*100)/4096);
            ui->label_IR4->clear();
//            ui->label_IR4->setText(QString::number(w.u16[3]));
            ui->progressBar_IR4->setValue((w.u16[3]*100)/4096);
            ui->label_IR5->clear();
//            ui->label_IR5->setText(QString::number(w.u16[4]));
            ui->progressBar_IR5->setValue((w.u16[4]*100)/4096);
            ui->label_IR6->clear();
//            ui->label_IR6->setText(QString::number(w.u16[5]));
            ui->progressBar_IR6->setValue((w.u16[5]*100)/4096);
            ui->label_IR7->clear();
//            ui->label_IR7->setText(QString::number(w.u16[6]));
            ui->progressBar_IR7->setValue((w.u16[6]*100)/4096);
            ui->label_IR8->clear();
//            ui->label_IR8->setText(QString::number(w.u16[7]));
            ui->progressBar_IR8->setValue((w.u16[7]*100)/4096);
        break;
        case 0xA1:
            if(RX[1] == 0x0D)
                ui->plainTextEdit->appendPlainText("MOTOR FUNCIONANDO");
        break;
        case 0xA2:
            if(RX[1] == 0x0D)
                ui->plainTextEdit->appendPlainText("POSICIONANDO");

            if(RX[1] == 0x0A)
                ui->plainTextEdit->appendPlainText("POSICIONADO - OK");
        break;
//        case 0xA3:                  //Ultrasonido
//            w.u8[0] = RX[1];
//            w.u8[1] = RX[2];
//            w.u8[2] = RX[3];
//            w.u8[3] = RX[4];
//            w.u32=(w.u32/(29.2*2));
//            ui->label_15->clear();
//            ui->label_15->setText(QString::number(w.u32));
//        break;
        case 0xA4:
            w.u8[0] = RX[1];
            w.u8[1] = RX[2];
            w.u8[2] = RX[3];
            w.u8[3] = RX[4];
            w.u32=((w.u32*2)/20)*60;
//            ui->label_19->clear();
//            ui->label_19->setText(QString::number(w.u32) + " RPM");
            w.u8[0] = RX[5];
            w.u8[1] = RX[6];
            w.u8[2] = RX[7];
            w.u8[3] = RX[8];
            w.u32=((w.u32*2)/20)*60;
//            ui->label_18->clear();
//            ui->label_18->setText(QString::number(w.u32) + " RPM");
        break;
        case 0xF0:
            if(RX[1] == 0x0D)
                ui->plainTextEdit->appendPlainText("I'M ALIVE"+QString::number(indR_UDP++));
        break;
//        case 0xF1:
//            while(RX[i]!= NULL){
//                aux += RX[i];
//                i++;
//            }

//            ui->label_11->clear();
//            ui->plainTextEdit->appendPlainText("Firmware: "+ aux);
//            ui->label_11->setText("Firmware: "+ aux);
//        break;
        case 0xFF:
            ui->plainTextEdit->appendPlainText("NO CMD");
        break;
    }
}

/*!
 * \brief MainWindow::on_pushButton_clicked
 * Función para definir parametros del puerto serie
 */

void MainWindow::on_pushButton_clicked()
{

    if(QSerialPort1->isOpen()){
        QSerialPort1->close();
        ui->pushButton->setText("OPEN");
//        ui->statusbar->showMessage("CLOSE - \"\"");
    }
    else{
        QSerialPort1->setPortName(ui->comboBox_2->currentText());
        QSerialPort1->setBaudRate(115200);
        QSerialPort1->setParity(QSerialPort::NoParity);
        QSerialPort1->setDataBits(QSerialPort::Data8);
        QSerialPort1->setStopBits(QSerialPort::OneStop);
        QSerialPort1->setFlowControl(QSerialPort::NoFlowControl);
        if(!QSerialPort1->open(QSerialPort::ReadWrite)){
            QMessageBox::information(this, "Serial PORT - ", "can't OPEN PORT(" + QSerialPort1->errorString() + ")");
            return;
        }
//        ui->statusbar->showMessage("OPEN - \"\"");
        ui->pushButton->setText("CLOSE");
    }
}

/*!
 * \brief MainWindow::on_pushButton_6_clicked
 * Función para limpiar el TextEdit
 */

void MainWindow::on_pushButton_6_clicked()
{
    ui->plainTextEdit->clear();
}

/*!
 * \brief MainWindow::on_pushButton_3_clicked
 * Función que detecta los puertos series disponibles y los carga en comboBox
 */

void MainWindow::on_pushButton_3_clicked()
{
    QStringList lista;
    const auto infos = QSerialPortInfo ::availablePorts();
    for (const QSerialPortInfo&info:infos){
        lista<<info.portName();
    }
    ui->comboBox_2->clear();
    ui->comboBox_2->addItems(lista);
}

/*!
 * \brief MainWindow::on_verticalSlider_2_valueChanged
 * \param value
 * Función para mostrar potencia MotorDerecho en un label
 */

//void MainWindow::on_verticalSlider_2_valueChanged(int value)
//{
//    ui->label_13->clear();
//    ui->label_13->setText(QString::number(value));
//}

///*!
// * \brief MainWindow::on_verticalSlider_valueChanged
// * \param value
// * Función para mostrar potencia MotorIzquierdo en un label
// */

//void MainWindow::on_verticalSlider_valueChanged(int value)
//{
//    ui->label_10->clear();
//    ui->label_10->setText(QString::number(value));
//}

///*!
// * \brief MainWindow::on_verticalSlider_4_valueChanged
// * \param value
// * Función para mostrar valor de angulo en un label
// */

//void MainWindow::on_verticalSlider_4_valueChanged(int value)
//{
//    uint8_t buf[24];
//    ui->label_14->clear();
//    ui->label_14->setText(QString::number(value));
//    buf[0] = 0xA2;
//    w.i8[0] = ui->verticalSlider_4->value();
//    buf[1] = w.i8[0];
//    SendCmd(buf,2);
//}

void MainWindow::on_comboBox_2_activated(const QString &arg1)
{

}

void MainWindow::on_pushButton_5_clicked()
{
    quint16 port;
    bool ok;

    if(QUdpSocket1->isOpen()){
        QUdpSocket1->abort();
        ui->pushButton_5->setText("CONECTAR");
    }
    else{
        port = ui->lineEdit->text().toInt(&ok);
        if(!ok)
            return;
        if(!QUdpSocket1->bind(port)){
               QMessageBox::warning(this,"UDP PORT","Can't BIND PORT");
               return;
        }

        QUdpSocket1->open(QUdpSocket::ReadWrite);

        ui->pushButton_5->setText("DESCONECTAR");
    }
}
