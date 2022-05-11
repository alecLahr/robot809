# Sends images or text to a specified email address

import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from datetime import datetime
from cv2 import imwrite
from constants import *


class Email():
    def __init__(self):
        self.smtpUser = 'alahrPi@gmail.com'
        self.smtpPass = 'raspberryLahr'


    def sendImage(self, img, pic_time=datetime.now().strftime('%Y%m%d%H%M%S'), recipients=DEFAULT_RECIPIENTS, subject=DEFAULT_SUBJECT):
        imwrite('imgs/email_out.jpg', img)

        msg = MIMEMultipart()
        msg['From'] = self.smtpUser
        # msg['To'] = recipients
        msg['To'] = 'alec.lahr@gmail.com'
        msg['Subject'] = subject
        msg.preamble = "Image recorded at " + pic_time + "\n"

        body = MIMEText("Image recorded at " + pic_time + "\n")
        msg.attach(body)

        fp = open('imgs/email_out.jpg', 'rb')
        img = MIMEImage(fp.read())
        fp.close()
        msg.attach(img)

        s = smtplib.SMTP('smtp.gmail.com', 587)
        s.ehlo()
        s.starttls()
        s.ehlo()

        s.login(self.smtpUser, self.smtpPass)
        s.sendmail(self.smtpUser, recipients, msg.as_string())
        s.quit()

        print("\nEmail delivered!\n")


    def sendText(self, pic_time=datetime.now().strftime('%Y%m%d%H%M%S'), recipients=DEFAULT_RECIPIENTS, subject=DEFAULT_SUBJECT):
        msg = MIMEMultipart()
        msg['From'] = self.smtpUser
        # msg['To'] = recipients
        msg['To'] = 'alec.lahr@gmail.com'
        msg['Subject'] = subject
        msg.preamble = "Block picked up at " + pic_time + "\n"

        body = MIMEText("Block picked up at " + pic_time + "\n")
        msg.attach(body)

        s = smtplib.SMTP('smtp.gmail.com', 587)
        s.ehlo()
        s.starttls()
        s.ehlo()

        s.login(self.smtpUser, self.smtpPass)
        s.sendmail(self.smtpUser, recipients, msg.as_string())
        s.quit()

        print("\nEmail delivered!\n")

        