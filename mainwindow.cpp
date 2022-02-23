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
    ui->comboBox->addItem("Test de motores", 0xA1);
    ui->comboBox->addItem("Constantes Kp,Ki,Kd", 0xE0);
    ui->comboBox->addItem("Parada de motores", 0xA2);


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

        DecodeHeader(buf, count);

        delete [] buf;
    }
}

/*!
 * \brief MainWindow::OnQTimer1
 * Función timer para control de flujo del programa
 */

void MainWindow::OnQTimer1(){

    if(time1 > 0){
        time1--;
    }
    if(!time1){
        time1 = 200;
    }

    if(header){
        timeout--;
        if(!timeout){
            header = 0;
        }
    }

}
/*!
 * \brief MainWindow::DecodeHeader
 * Funcion para decodificar la cabecera de los paquetes entrantes
 */
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
    uint8_t tx[24], cks, i;
    cks = 0;
    i = 0;

    if((!QSerialPort1->isOpen()) && (!QUdpSocket1->isOpen()))
        return;
    tx[0] = 'U';
    tx[1] = 'N';
    tx[2] = 'E';
    tx[3] = 'R';
    tx[4] = length+1;
    tx[5] = ':';

    for(i=0; i<length; i++){
        tx[6+i] = buf[i];
    }

    for (i=0; i<(length+6); i++) {
        cks ^= tx[i];
    }

    tx[i] = cks;
    if(QSerialPort1->isOpen()){
        QSerialPort1->write((char *)tx, length+7);
    }
    if(QUdpSocket1->isOpen()){
        QUdpSocket1->writeDatagram((char *) tx,length+7,hostAddress,hostPort);
    }
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
    if(!time1){
        return;
    }
    else{
        cmd = ui->comboBox->currentData().toInt();
        ui->plainTextEdit->appendPlainText("0x" + (QString("%1").arg(cmd, 2, 16, QChar('0'))).toUpper());

        switch (cmd) {
            case 0xF0:                  //ALIVE
            ui->plainTextEdit->appendPlainText("ALIVE???");
                buf[0] = cmd;
                SendCmd(buf,1);
            break;

            case 0xA1:
                buf[0] = cmd;
                w.i16[0] = ui->verticalSlider->value();            //Motor izquierdo
                buf[1]= w.i8[0];
                buf[2]= w.i8[1];
                w.i16[1] = ui->verticalSlider_2->value();          //Motor derecho
                buf[3]= w.i8[2];
                buf[4]= w.i8[3];
                SendCmd(buf,5);
            break;

            case 0xA2:
                buf[0] = cmd;
                SendCmd(buf,1);
            break;

            case 0xE0:
                buf[0] = cmd;
                //Kp motor izquierdo
                buf[1] = ui->spinBox->value();
                //Kd motor izquierdo
                buf[2] = ui->spinBox_3->value();

                //Kp motor derecho
                buf[3] = ui->spinBox_2->value();
                //Kd motor derecho
                buf[4] = ui->spinBox_4->value();
                SendCmd(buf,5);
            break;

        }
    }
}

/*!
 * \brief MainWindow::DecodeCmd
 * \param RX -> buffer de recepcion MBED->PC
 * Función para decodificar el comando+payload
 */

void MainWindow::DecodeCmd(uint8_t *RX){
    uint8_t buf[1];
    QString aux;
    switch(RX[0]){

        case 0xA7:
            buf[0] = 0xA7;
            SendCmd(buf,1);
            //Timer cada 10ms
            QTimer1->start(1);
            time1 = 210;
            FIRSTALIVE = 1;
            ui->plainTextEdit->appendPlainText("PC CONECTADA");
        break;

        case 0xA8:

            w.u8[0] = RX[1];
            w.u8[1] = RX[2];
            w.u8[2] = RX[3];
            w.u8[3] = RX[4];
            ui->label_IR1->clear();
            ui->label_IR1->setText(QString::number(w.u16[0]));
            ui->progressBar_IR1->setValue((w.u16[0]*100)/4096);
            ui->label_IR2->clear();
            ui->label_IR2->setText(QString::number(w.u16[1]));
            ui->progressBar_IR2->setValue((w.u16[1]*100)/4096);
            w.u8[0] = RX[5];
            w.u8[1] = RX[6];
            w.u8[2] = RX[7];
            w.u8[3] = RX[8];
            ui->label_IR3->clear();
            ui->label_IR3->setText(QString::number(w.u16[0]));
            ui->progressBar_IR3->setValue((w.u16[0]*100)/4096);
            ui->label_IR4->clear();
            ui->label_IR4->setText(QString::number(w.u16[1]));
            ui->progressBar_IR4->setValue((w.u16[1]*100)/4096);
            w.u8[0] = RX[9];
            w.u8[1] = RX[10];
            w.u8[2] = RX[11];
            w.u8[3] = RX[12];
            ui->label_IR5->clear();
            ui->label_IR5->setText(QString::number(w.u16[0]));
            ui->progressBar_IR5->setValue((w.u16[0]*100)/4096);
            ui->label_IR6->clear();
            ui->label_IR6->setText(QString::number(w.u16[1]));
            ui->progressBar_IR6->setValue((w.u16[1]*100)/4096);
            w.u8[0] = RX[13];
            w.u8[1] = RX[14];
            w.u8[2] = RX[15];
            w.u8[3] = RX[16];
            ui->label_IR7->clear();
            ui->label_IR7->setText(QString::number(w.u16[0]));
            ui->progressBar_IR7->setValue((w.u16[0]*100)/4096);
            ui->label_IR8->clear();
            ui->label_IR8->setText(QString::number(w.u16[1]));
            ui->progressBar_IR8->setValue((w.u16[1]*100)/4096);
//ERROR
            w.i8[0] = RX[17];
            w.i8[1] = RX[18];
            ui->lineEdit_4->clear();
            ui->lineEdit_4->setText(QString::number(w.i16[0]));
//PWMA
            w.i8[0] = RX[19];
            w.i8[1] = RX[20];
            ui->lineEdit_5->clear();
            ui->lineEdit_5->setText(QString::number(w.i16[0]));
//PWMB
            w.i8[0] = RX[21];
            w.i8[1] = RX[22];
            ui->lineEdit_6->clear();
            ui->lineEdit_6->setText(QString::number(w.i16[0]));
//PWM_A
            w.i8[0] = RX[23];
            w.i8[1] = RX[24];
            ui->lineEdit_7->clear();
            ui->lineEdit_7->setText(QString::number(w.i16[0]));
//PWM_B
            w.i8[0] = RX[25];
            w.i8[1] = RX[26];
            ui->lineEdit_10->clear();
            ui->lineEdit_10->setText(QString::number(w.i16[0]));
//BASEa
            w.i8[0] = RX[27];
            w.i8[1] = RX[28];
            ui->lineEdit_8->clear();
            ui->lineEdit_8->setText(QString::number(w.i16[0]));
//BASEb
            w.i8[0] = RX[29];
            w.i8[1] = RX[30];
            ui->lineEdit_9->clear();
            ui->lineEdit_9->setText(QString::number(w.i16[0]));
//PROP
            w.i8[0] = RX[31];
            w.i8[1] = RX[32];
            ui->lineEdit_11->clear();
            ui->lineEdit_11->setText(QString::number(w.i16[0]));

            Counter++;
            if(Counter>=5){
                if(RX[33] == 0xF0 && RX[34] == 0x0D){
                    ui->plainTextEdit->appendPlainText("I'M ALIVE "+QString::number(Counter_2++));
                }
                Counter = 0;
            }
        break;

        case 0xA1:
            if(RX[1] == 0x0D)
                ui->plainTextEdit->appendPlainText("MOTOR FUNCIONANDO");
        break;

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
        ui->pushButton->setText("ABRIR");
        ui->pushButton->setStyleSheet("background-color:red");
    }
    else{
        QSerialPort1->setPortName(ui->comboBox_2->currentText());
        QSerialPort1->setBaudRate(115200);
        QSerialPort1->setParity(QSerialPort::NoParity);
        QSerialPort1->setDataBits(QSerialPort::Data8);
        QSerialPort1->setStopBits(QSerialPort::OneStop);
        QSerialPort1->setFlowControl(QSerialPort::NoFlowControl);
        if(!QSerialPort1->open(QSerialPort::ReadWrite)){
            QMessageBox::information(this, "Puerto serie - ", "no se pudo abrir el puerto(" + QSerialPort1->errorString() + ")");
            return;
        }
        ui->pushButton->setText("CERRAR");
        ui->pushButton->setStyleSheet("background-color:green");
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

void MainWindow::on_verticalSlider_2_valueChanged(int value)
{
    ui->lineEdit_3->clear();
    ui->lineEdit_3->setText(QString::number(value));
}

/*!
 * \brief MainWindow::on_verticalSlider_valueChanged
 * \param value
 * Función para mostrar potencia MotorIzquierdo en un label
 */

void MainWindow::on_verticalSlider_valueChanged(int value)
{
    ui->lineEdit_2->clear();
    ui->lineEdit_2->setText(QString::number(value));
}

void MainWindow::on_pushButton_5_clicked()
{
    uint8_t buf[1];
    quint16 port;
    bool ok;

    if(QUdpSocket1->isOpen()){
        buf[0] = 0xA6;
        QUdpSocket1->abort();
        ui->pushButton_5->setText("ABRIR");
        SendCmd(buf, 1);
        FIRSTALIVE = 0;
        clean();
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

        ui->pushButton_5->setText("CERRAR");

    }
}

void MainWindow::on_lineEdit_2_editingFinished()
{
    ui->verticalSlider->setValue(ui->lineEdit_2->text().toInt());
}

void MainWindow::on_lineEdit_3_editingFinished()
{
    ui->verticalSlider_2->setValue(ui->lineEdit_3->text().toInt());
}

void MainWindow::clean(){
    ui->label_12->clear();
    ui->label_IR1->clear();
    ui->label_IR2->clear();
    ui->label_IR3->clear();
    ui->label_IR4->clear();
    ui->label_IR5->clear();
    ui->label_IR6->clear();
    ui->label_IR7->clear();
    ui->label_IR8->clear();
    ui->progressBar_IR1->reset();
    ui->progressBar_IR2->reset();
    ui->progressBar_IR3->reset();
    ui->progressBar_IR4->reset();
    ui->progressBar_IR5->reset();
    ui->progressBar_IR6->reset();
    ui->progressBar_IR7->reset();
    ui->progressBar_IR8->reset();
    ui->lineEdit_4->clear();
    ui->lineEdit_5->clear();
    ui->lineEdit_6->clear();
    ui->lineEdit_7->clear();
}

void MainWindow::on_pushButton_2_clicked()
{
    uint8_t buf[1];

    if(!time1){
        return;
    }
    else{
        if(ui->pushButton_2->text() == "START"){
            buf[0] = 0xA3;
            SendCmd(buf,1);
            ui->pushButton_2->setText("STOP");
        }
        else{
            buf[0] = 0xA2;
            SendCmd(buf,1);
            ui->pushButton_2->setText("START");
        }

        ui->plainTextEdit->appendPlainText("0x" + (QString("%1").arg(buf[0], 2, 16, QChar('0'))).toUpper());
    }
}
