import datetime as dt


edital = 'http://www.senaicimatec.com.br/wp-content/uploads/2021/02/Processo-Seletivo-Bolsas-Tec-do-Centro-de-Comp-de-Robotica-e-Sistemas-Autonomos.pdf'
atualizacoes = 'http://www.senaicimatec.com.br/processos/programas-de-bolsas-cientificas/#/editais'
rep = 'https://github.com/Brazilian-Institute-of-Robotics/desafiorobotica'

data_final_inscriçao = dt.date(2021, 3, 8)
data_final_rep = dt.date(2021, 3, 12)
today = dt.date.today()

#print(f'Tempo para inscrição {data_final_inscriçao - today}')
print(f'Tempo para envio do rep. {data_final_rep - today}')
