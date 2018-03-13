from django.shortcuts import render
from django.shortcuts import redirect
from django.http import HttpResponse
from django.views.decorators.csrf import csrf_exempt
import json
import urllib3


# Create your views here.
def index(request):

	return render(request, 'index.html')


@csrf_exempt
def jsonhandler(request):
	if request.method == 'POST':

		IP = "http://192.168.1.114"

		# Write to USB through serial
		url = IP + request.POST.controls;
		print(url)
		http = urllib3.PoolManager()
		r = http.request('GET', url)

		return JsonResponse(r.data.decode('utf-8'))

	elif request.method == 'GET':
		return redirect('/simulation')