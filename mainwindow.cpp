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
    ui->comboBox->addItem("Constantes Kp,Ki,Kd", 0xE2);
    ui->comboBox->addItem("Parada de motores", 0xA2);

//Timer cada 10ms
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

        DecodeHeader(buf, count);

        delete [] buf;
    }
}

/*!
 * \brief MainWindow::OnQTimer1
 * Función timer para control de flujo del programa
 */

void MainWindow::OnQTimer1(){
    uint8_t buf[24];

    time1 ++;

    if(header){
        timeout--;
        if(!timeout){
            header = 0;
        }
    }
    if(time1 >= 200){
        if((QUdpSocket1->isOpen()) && (FIRSTALIVE)){
            switch (Dato) {
                case 0:
                    buf[0] = 0xA0;
                    SendCmd(buf,1);
                    Dato++;
                    time1 = 0;
                break;

                case 1:
                    buf[0] = 0xF0;
                    SendCmd(buf,1);
                    Dato = 0;
                    time1 = 0;
                break;
            }
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
            w.u32 = ui->verticalSlider->value();            //Motor izquierdo
            buf[1]= w.u8[0];
            buf[2]= w.u8[1];
            buf[3]= w.u8[2];
            buf[4]= w.u8[3];
            w.u32 = ui->verticalSlider_2->value();          //Motor derecho
            buf[5]= w.u8[0];
            buf[6]= w.u8[1];
            buf[7]= w.u8[2];
            buf[8]= w.u8[3];
            SendCmd(buf,9);
        break;

    }
}

/*!
 * \brief MainWindow::DecodeCmd
 * \param RX -> buffer de recepcion MBED->PC
 * Función para decodificar el comando+payload
 */

void MainWindow::DecodeCmd(uint8_t *RX){
    QString aux;
    switch(RX[0]){
        case 0xA0:
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
//            w.u8[0] = RX[1];
//            w.u8[1] = RX[2];
//            w.u8[2] = RX[3];
//            w.u8[3] = RX[4];
//            w.u8[4] = RX[5];
//            w.u8[5] = RX[6];
//            w.u8[6] = RX[7];
//            w.u8[7] = RX[8];
//            w.u8[8] = RX[9];
//            w.u8[9] = RX[10];
//            w.u8[10] = RX[11];
//            w.u8[11] = RX[12];
//            w.u8[12] = RX[13];
//            w.u8[13] = RX[14];
//            w.u8[14] = RX[15];
//            w.u8[15] = RX[16];
//            ui->label_IR1->clear();
//            ui->label_IR1->setText(QString::number(w.u16[0]));
//            ui->progressBar_IR1->setValue((w.u16[0]*100)/4096);
//            ui->label_IR2->clear();
//            ui->label_IR2->setText(QString::number(w.u16[1]));
//            ui->progressBar_IR2->setValue((w.u16[1]*100)/4096);
//            ui->label_IR3->clear();
//            ui->label_IR3->setText(QString::number(w.u16[2]));
//            ui->progressBar_IR3->setValue((w.u16[2]*100)/4096);
//            ui->label_IR4->clear();
//            ui->label_IR4->setText(QString::number(w.u16[3]));
//            ui->progressBar_IR4->setValue((w.u16[3]*100)/4096);
//            ui->label_IR5->clear();
//            ui->label_IR5->setText(QString::number(w.u16[4]));
//            ui->progressBar_IR5->setValue((w.u16[4]*100)/4096);
//            ui->label_IR6->clear();
//            ui->label_IR6->setText(QString::number(w.u16[5]));
//            ui->progressBar_IR6->setValue((w.u16[5]*100)/4096);
//            ui->label_IR7->clear();
//            ui->label_IR7->setText(QString::number(w.u16[6]));
//            ui->progressBar_IR7->setValue((w.u16[6]*100)/4096);
//            ui->label_IR8->clear();
//            ui->label_IR8->setText(QString::number(w.u16[7]));
//            ui->progressBar_IR8->setValue((w.u16[7]*100)/4096);
        break;
        case 0xA1:
            if(RX[1] == 0x0D)
                ui->plainTextEdit->appendPlainText("MOTOR FUNCIONANDO");
        break;

        case 0xF0:
            if(RX[1] == 0x0D){
                ui->plainTextEdit->appendPlainText("I'M ALIVE "+QString::number(Counter++));
                FIRSTALIVE = 1;
            }
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
    quint16 port;
    bool ok;

    if(QUdpSocket1->isOpen()){
        QUdpSocket1->abort();
        ui->pushButton_5->setText("ABRIR");

        ui->label_12->clear();
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
